using UnityEngine;

public class LongitudinalFlight : MonoBehaviour
{
    [Header("Aircraft Constants")]
    public float mass = 1000f;
    public float Iy = 2000f; 
    public float wingArea = 25f;
    public float chord = 2f; 
    public float rho = 1.225f; 

    [Header("Aerodynamic Curves")]
    public AnimationCurve ClCurve = AnimationCurve.Linear(-20, -0.5f, 20, 1.5f);
    public AnimationCurve CdCurve = AnimationCurve.Linear(-20, -0.5f, 20, 1.5f);
    public AnimationCurve CmCurve = AnimationCurve.Linear(-20, 0.2f, 20, -0.2f);

    [Header("EKF 6-State Estimate")]
    public float u, w, q, theta, posX, posZ;

    [Header("Ground Truth (Actual Physics)")]
    public float trueU = 50f;
    public float trueW, trueQ, trueTheta;
    // We use transform.position for truePosX and truePosZ

    [Header("Kalman Filter Settings")]
    public float processNoise = 0.05f; 
    public float sensorNoise = 1.5f;
    public float IMUsensorNoise = 0.1f; 
    
    private float[,] P = new float[6, 6];
    private float[,] Q = new float[6, 6];
    private float[,] R = new float[6, 6];

    [Header("Control")]
    public float throttle; 
    public float elevator; 
    public float elevatorEffectiveness = 0.1f;

    public float[] z = new float[6]; // Sensor measurements

    private Rigidbody rb;

    void Start() {
        rb = GetComponent<Rigidbody>();
        u = 50f; 
        
        // Initialize Covariance Matrices
        for(int i=0; i<6; i++) {
            P[i,i] = 0.1f;
            Q[i,i] = processNoise;
            R[i,i] = sensorNoise;
        }
        // Gyros (q, theta) are usually more precise than GPS/Pitot
        R[2,2] *= IMUsensorNoise; 
        R[3,3] *= IMUsensorNoise;
    }

    void FixedUpdate() {
        float dt = Time.fixedDeltaTime;

        // 1. UPDATE REAL WORLD
        UpdateGroundTruth(dt);

        // 2. EKF STEP 1: PREDICTION
        // Predict state using physics equations
        float[] statePred = PredictState(u, w, q, theta, posX, posZ, dt);
        
        // Predict Covariance: P = F*P*F^T + Q
        float[,] F = CalculateJacobian6x6(u, w, q, theta, dt);
        float[,] FT = MatrixTranspose(F);
        P = MatrixAdd(MatrixMultiply(F, MatrixMultiply(P, FT)), Q);

        // 3. EKF STEP 2: MEASUREMENT
        z = GetNoisySensorData();

        // 4. EKF STEP 3: CORRECTION
        // Innovation: y = z - Hx (H is identity here as we measure all states)
        float[] y = new float[6];
        for (int i = 0; i < 6; i++) y[i] = z[i] - statePred[i];

        // Innovation Covariance: S = P + R
        float[,] S = MatrixAdd(P, R);
        float[,] S_inv = MatrixInverse6x6(S);

        // Kalman Gain: K = P * S^-1
        float[,] K = MatrixMultiply(P, S_inv);

        // Update State Estimate
        float[] correction = VectorMultiply(K, y);
        u = statePred[0] + correction[0];
        w = statePred[1] + correction[1];
        q = statePred[2] + correction[2];
        theta = statePred[3] + correction[3];
        posX = statePred[4] + correction[4];
        posZ = statePred[5] + correction[5];

        // Update Covariance: P = (I - K) * P
        float[,] I = Identity6x6();
        P = MatrixMultiply(MatrixSubtract(I, K), P);

        // 5. RENDER THE PHYSICAL PLANE
        ApplyToRigidbody(trueU, trueW, trueQ, trueTheta);
    }

    float[] PredictState(float cu, float cw, float cq, float ct, float cx, float cz, float dt) {
        float alpha = Mathf.Atan2(cw, cu) * Mathf.Rad2Deg;
        float V_sq = cu * cu + cw * cw;
        float q_dyn = 0.5f * rho * V_sq;

        float lift = ClCurve.Evaluate(alpha) * q_dyn * wingArea;
        float drag = CdCurve.Evaluate(alpha) * q_dyn * wingArea;
        float moment = (CmCurve.Evaluate(alpha) + (elevator * elevatorEffectiveness)) 
                        * q_dyn * wingArea * chord + (cq * -1000f);

        float uDot = ((throttle * 5000f) - drag) / mass - (cq * cw) - (9.81f * Mathf.Sin(ct));
        float wDot = (-lift / mass) + (cq * cu) + (9.81f * Mathf.Cos(ct));
        float qDot = moment / Iy;

        return new float[] {
            cu + uDot * dt,
            cw + wDot * dt,
            cq + qDot * dt,
            ct + cq * dt,
            cx + (cu * Mathf.Cos(ct) + cw * Mathf.Sin(ct)) * dt,
            cz + (cu * Mathf.Sin(ct) - cw * Mathf.Cos(ct)) * dt
        };
    }

    void UpdateGroundTruth(float dt) {
        float alpha = Mathf.Atan2(trueW, trueU) * Mathf.Rad2Deg;
        float q_dyn = 0.5f * rho * (trueU * trueU + trueW * trueW);

        float lift = ClCurve.Evaluate(alpha) * q_dyn * wingArea;
        float drag = CdCurve.Evaluate(alpha) * q_dyn * wingArea;
        float moment = (CmCurve.Evaluate(alpha) + (elevator * elevatorEffectiveness)) 
                        * q_dyn * wingArea * chord + (trueQ * -1000f);

        float uDot = ((throttle * 5000f) - drag) / mass - (trueQ * trueW) - (9.81f * Mathf.Sin(trueTheta));
        float wDot = (-lift / mass) + (trueQ * trueU) + (9.81f * Mathf.Cos(trueTheta));
        float qDot = moment / Iy;

        trueU += uDot * dt;
        trueW += wDot * dt;
        trueQ += qDot * dt;
        trueTheta += trueQ * dt;
    }

    public float[] GetNoisySensorData() {
        return new float[] {
            trueU + GenerateGaussianNoise(sensorNoise),
            trueW + GenerateGaussianNoise(sensorNoise),
            trueQ + GenerateGaussianNoise(sensorNoise * 0.1f),
            trueTheta + GenerateGaussianNoise(sensorNoise * 0.1f),
            transform.position.z + GenerateGaussianNoise(sensorNoise * 2f), // Map Z to World X
            -transform.position.y + GenerateGaussianNoise(sensorNoise * 2f) // Map -Y to World Z
        };
    }

    float GenerateGaussianNoise(float stdDev) {
        float u1 = Random.value; float u2 = Random.value;
        return stdDev * Mathf.Sqrt(-2.0f * Mathf.Log(u1)) * Mathf.Sin(2.0f * Mathf.PI * u2);
    }

    void ApplyToRigidbody(float u, float w, float q, float theta) {
        Vector3 bodyVelocity = new Vector3(0, -w, u);
        rb.velocity = transform.TransformDirection(bodyVelocity);
        transform.rotation = Quaternion.Euler(theta * Mathf.Rad2Deg, 0, 0);
    }

    // --- 6x6 MATRIX MATH HELPERS ---

    public float[,] CalculateJacobian6x6(float cu, float cw, float cq, float ct, float dt) {
        float[,] J = Identity6x6();
        float alpha = Mathf.Atan2(cw, cu) * Mathf.Rad2Deg;
        float dCl = (ClCurve.Evaluate(alpha + 0.1f) - ClCurve.Evaluate(alpha - 0.1f)) / 0.2f;
        
        float Xu = -(rho * cu * wingArea * 0.05f) / mass;
        float Zw = -(0.5f * rho * cu * wingArea * dCl) / mass;

        // u, w, q, theta rows
        J[0,0] += Xu * dt; J[0,1] = -cq * dt; J[0,3] = -9.81f * Mathf.Cos(ct) * dt;
        J[1,0] = cq * dt; J[1,1] += Zw * dt; J[1,3] = -9.81f * Mathf.Sin(ct) * dt;
        J[2,2] += (-1000f/Iy) * dt;
        J[3,2] = dt;

        // Position rows (x, z)
        J[4,0] = Mathf.Cos(ct) * dt; J[4,1] = Mathf.Sin(ct) * dt;
        J[4,3] = (-cu * Mathf.Sin(ct) + cw * Mathf.Cos(ct)) * dt;
        J[5,0] = Mathf.Sin(ct) * dt; J[5,1] = -Mathf.Cos(ct) * dt;
        J[5,3] = (cu * Mathf.Cos(ct) + cw * Mathf.Sin(ct)) * dt;

        return J;
    }

    float[,] MatrixMultiply(float[,] A, float[,] B) {
        float[,] C = new float[6, 6];
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
                for (int k = 0; k < 6; k++)
                    C[i, j] += A[i, k] * B[k, j];
        return C;
    }

    float[] VectorMultiply(float[,] M, float[] V) {
        float[] res = new float[6];
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
                res[i] += M[i, j] * V[j];
        return res;
    }

    float[,] MatrixTranspose(float[,] A) {
        float[,] res = new float[6, 6];
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
                res[j, i] = A[i, j];
        return res;
    }

    float[,] MatrixAdd(float[,] A, float[,] B) {
        float[,] res = new float[6, 6];
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
                res[i, j] = A[i, j] + B[i, j];
        return res;
    }

    float[,] MatrixSubtract(float[,] A, float[,] B) {
        float[,] res = new float[6, 6];
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
                res[i, j] = A[i, j] - B[i, j];
        return res;
    }

    float[,] Identity6x6() {
        float[,] res = new float[6, 6];
        for (int i = 0; i < 6; i++) res[i, i] = 1f;
        return res;
    }

    // Simplified Inverse for 6x6 (Diagonal dominance assumed for stability)
    float[,] MatrixInverse6x6(float[,] m) {
        // In a real EKF you'd use Cholesky decomposition, but for a game
        // a simple Gauss-Jordan or checking diagonal works if dt is small.
        float[,] res = Identity6x6();
        float[,] a = (float[,])m.Clone();
        for (int i = 0; i < 6; i++) {
            float v = a[i, i];
            for (int j = 0; j < 6; j++) {
                a[i, j] /= v;
                res[i, j] /= v;
            }
            for (int k = 0; k < 6; k++) {
                if (k != i) {
                    float factor = a[k, i];
                    for (int j = 0; j < 6; j++) {
                        a[k, j] -= factor * a[i, j];
                        res[k, j] -= factor * res[i, j];
                    }
                }
            }
        }
        return res;
    }
}