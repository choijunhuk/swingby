using System;
using UnityEngine;

[DisallowMultipleComponent]
public class SimulationManager : MonoBehaviour
{
    public static SimulationManager Instance { get; private set; }

    [Header("Prefab & References")]
    [SerializeField] private GameObject shipPrefab;
    [SerializeField] private CelestialBody planetBody;
    [SerializeField] private Transform planetTransform;

    [Header("우주선 파라미터 (UI 입력, 단위: km)")]
    public float shipMass = 1e4f;
    public Vector3 shipPos = new Vector3(-5000f, 0f, 0f);
    public Vector3 shipVel = new Vector3(1f, 1f, 0f);
    public float shipRadius = 0.01f;

    [Header("행성 파라미터 (UI 입력, 단위: km)")]
    public float planetMass = 5.972e24f;
    public Vector3 planetPos = new Vector3(1000f, 0f, 0f);
    public Vector3 planetVel = new Vector3(0f, 5f, 0f);
    public float planetRadius = 0f;
    public bool isPointMass = true;

    [Header("물리 상수 & 설정")]
    public float gravConst = 6.674e-11f;
    public float timeScale = 3.0f;
    public float trajWidth = 0.1f;

    [Header("단위 스케일 (km → Unity 단위)")]
    [Tooltip("몇으로 나눠서 Unity 좌표(u)로 변환할지 입력하세요. 예: 100 (1유닛=100km)")]
    public float unitScale = 100f;

    [Header("해석적 vs 수치 통합")]
    [Tooltip("0: 해석적, 1: 수치")]
    public int simulationMethod = 0;

    [Header("Trail Colors")]
    public Color shipTrailColor = Color.cyan;
    public Color planetTrailColor = Color.yellow;

    [Header("카메라 모드")]
    public int camMode = 0;

    public event Action<string> OnLogUpdated;
    public event Action<float> OnSpeedUpdated;

    private GameObject shipInstance;
    private TrailRenderer shipTrail, planetTrail;
    private float mu; // km³/s²
    private float a;
    private float e;
    private float omega;
    private float theta;
    private float r0Mag;
    private Vector3 r0;
    private Vector3 v0;
    private float M0;
    private float n;
    private float elapsedTime;
    private bool running;
    private bool analyticEllipse;
    private bool firstFrame; // 첫 프레임 스킵용
    private Vector3 pos_m; // km 단위
    private Vector3 vel_m; // km/s 단위
    private Vector3 planetPos_m; // km 단위

    public bool IsRunning => running;
    public Transform ShipTransform => shipInstance?.transform;
    public Transform PlanetTransform => planetTransform;

    void Awake()
    {
        if (Instance == null) Instance = this;
        else Destroy(gameObject);
        Debug.Log("[SimulationManager] Awake: Singleton 초기화 완료");
    }

    public void StartSimulation()
    {
        ClearPrevious();
        elapsedTime = 0f;
        analyticEllipse = false;
        firstFrame = true; // 첫 프레임에는 궤도 계산 건너뛰기

        // ─────────────────────────────────────────────────────────────
        // mu 계산: G (m³/(kg·s²)) × planetMass (kg) → m³/s²
        //   → 1 km = 1000 m 이므로 (km³/s²)로 바꾸려면 1e-9를 곱함
        //   (m³/s²) × (1 km/1000 m)³ = (m³/s²) × 1e-9 = km³/s²
        mu = gravConst * planetMass * 1e-9f; // km³/s²
        // (이전과 동일하게, 내부 계산은 모두 km·s·kg 단위. Unity 좌표로 변환 시에만 unitScale 적용됨)
        // ─────────────────────────────────────────────────────────────

        planetPos_m = planetPos; // km 단위
        planetTransform.position = planetPos_m / unitScale; // km → Unity 단위
        planetTransform.localScale = Vector3.one;
        Debug.Log($"[SimulationManager] StartSimulation: planetPos={planetPos}, planetPos_m={planetPos_m}, planetTransform.position={planetTransform.position}, planetVel={planetVel}, planetMass={planetMass}, planetRadius={planetRadius}, unitScale={unitScale}");

        if (!planetBody.TryGetComponent<TrailRenderer>(out planetTrail))
            planetTrail = planetBody.gameObject.AddComponent<TrailRenderer>();
        planetTrail.widthMultiplier = trajWidth;
        planetTrail.time = 100f;
        planetTrail.minVertexDistance = 0.2f;
        planetTrail.startColor = planetTrail.endColor = planetTrailColor;
        planetTrail.Clear();
        planetTrail.emitting = true;

        shipInstance = Instantiate(shipPrefab);
        if (shipInstance == null) Debug.LogError("[SimulationManager] 우주선 생성 실패!");
        shipInstance.transform.localScale = Vector3.one;
        pos_m = shipPos; // km 단위
        shipInstance.transform.position = pos_m / unitScale; // km → Unity 단위
        vel_m = shipVel; // km/s 단위
        Debug.Log($"[SimulationManager] 우주선 생성: shipMass={shipMass}, shipPos={shipPos}, shipVel={shipVel}, shipRadius={shipRadius}, shipTransform.position={shipInstance.transform.position}");

        if (!shipInstance.TryGetComponent<TrailRenderer>(out shipTrail))
            shipTrail = shipInstance.AddComponent<TrailRenderer>();
        shipTrail.widthMultiplier = trajWidth;
        shipTrail.time = 100f;
        shipTrail.minVertexDistance = 0.2f;
        shipTrail.startColor = shipTrail.endColor = shipTrailColor;
        shipTrail.Clear();
        shipTrail.emitting = true;

        Collider shipCol = shipInstance.GetComponent<Collider>();
        Collider planetCol = planetBody.GetComponent<Collider>();
        if (shipCol != null && planetCol != null)
            Physics.IgnoreCollision(shipCol, planetCol, true);

        if (simulationMethod == 0)
        {
            r0 = shipPos - planetPos_m; // km 단위
            v0 = shipVel - planetVel;   // km/s 단위
            r0Mag = r0.magnitude;

            float v0sq = v0.sqrMagnitude;
            float energy = 0.5f * v0sq - mu / r0Mag;

            float Lmag = Vector3.Cross(r0, v0).magnitude;
            if (Lmag < 1e-6f)
            {
                analyticEllipse = false;
                running = true;
                OnLogUpdated?.Invoke("--- 진동 또는 방사형 궤도: 수치 모드로 전환 ---");
                Debug.Log("[SimulationManager] 진동/방사형 궤도: 수치 모드로 전환");
            }
            else if (energy < 0f)
            {
                // 타원 궤도 해석적 계산 준비
                analyticEllipse = true;
                Vector3 eVec = ((v0sq - mu / r0Mag) * r0 - Vector3.Dot(r0, v0) * v0) / mu;
                e = eVec.magnitude;

                float rawOmega = Mathf.Atan2(eVec.y, eVec.x);
                if (rawOmega < 0f) rawOmega += 2f * Mathf.PI;
                omega = rawOmega;

                a = -mu / (2f * energy);
                n = Mathf.Sqrt(mu / (a * a * a));

                float cosTheta0 = Vector3.Dot(eVec, r0) / (e * r0Mag);
                cosTheta0 = Mathf.Clamp(cosTheta0, -1f, 1f);
                float theta0 = Mathf.Acos(cosTheta0);
                if (Vector3.Dot(r0, v0) < 0f)
                    theta0 = 2f * Mathf.PI - theta0;

                float tanHalfTheta0 = Mathf.Tan(theta0 / 2f);
                float sqrtFactor = Mathf.Sqrt((1f - e) / (1f + e));
                float E0 = 2f * Mathf.Atan(sqrtFactor * tanHalfTheta0);
                if (E0 < 0f) E0 += 2f * Mathf.PI;

                M0 = E0 - e * Mathf.Sin(E0);
                theta = theta0;

                running = true;
                OnLogUpdated?.Invoke("--- 시뮬레이션 시작 (이론적·타원 궤도) ---");
                Debug.Log($"[SimulationManager] 시뮬레이션 시작 (이론적·타원 궤도): a={a}, e={e}, omega={omega}");
            }
            else
            {
                analyticEllipse = false;
                running = true;
                OnLogUpdated?.Invoke("--- 에너지 ≥ 0: 수치 모드로 전환 ---");
                Debug.Log("[SimulationManager] 에너지 ≥ 0: 수치 모드로 전환");
            }
        }
        else
        {
            analyticEllipse = false;
            running = true;
            OnLogUpdated?.Invoke("--- 시뮬레이션 시작 (수치) ---");
            Debug.Log("[SimulationManager] 시뮬레이션 시작 (수치)");
        }
    }

    public void StopSimulation()
    {
        running = false;
        if (shipTrail != null) shipTrail.emitting = false;
        if (planetTrail != null) planetTrail.emitting = false;
        OnLogUpdated?.Invoke("--- 시뮬레이션 정지 ---");
        Debug.Log("[SimulationManager] 시뮬레이션 정지");
    }

    public void ResumeSimulation()
    {
        running = true;
        if (shipTrail != null) shipTrail.emitting = true;
        if (planetTrail != null) planetTrail.emitting = true;
        OnLogUpdated?.Invoke("--- 시뮬레이션 재개됨 ---");
        Debug.Log("[SimulationManager] 시뮬레이션 재개됨");
    }

    private Vector3 CalculateAcceleration(Vector3 position, Vector3 velocity)
    {
        Vector3 rVec = planetPos_m - position; // km 단위
        float distSqr = rVec.sqrMagnitude;
        if (distSqr > 1e-12f && !float.IsNaN(distSqr))
        {
            float dist = Mathf.Sqrt(distSqr);
            // 중력 가속도: μ / r²
            return (mu / (dist * dist)) * rVec.normalized; // km/s²
        }
        Debug.LogWarning("[SimulationManager] CalculateAcceleration: Invalid distSqr=" + distSqr);
        return Vector3.zero;
    }

    void FixedUpdate()
    {
        if (!running || shipInstance == null) return;

        // 1) 매 프레임 실제 흐르는 물리시간(초) → scaledDt로 변환
        float realDt = Time.fixedDeltaTime;          // Unity 물리 업데이트 주기(초)
        float scaledDt = realDt * timeScale;         // 사용자 지정 시간 배율 적용(초)

        // ─────────────────────────────────────────────────────────────
        // “첫 프레임” 조건: 해석적 타원 계산을 바로 하지 않도록 한 번 건너뜀
        if (firstFrame && simulationMethod == 0 && analyticEllipse)
        {
            firstFrame = false;
            elapsedTime += scaledDt;
            // 행성 위치는 계속 업데이트만 해 둠
            planetPos_m += planetVel * scaledDt;
            planetTransform.position = planetPos_m / unitScale;
            return;
        }
        // ─────────────────────────────────────────────────────────────

        elapsedTime += scaledDt;

        // 2) 행성 위치 업데이트 (항상)
        planetPos_m += planetVel * scaledDt;       // km 단위
        planetTransform.position = planetPos_m / unitScale; // Unity 좌표

        if (simulationMethod == 0 && analyticEllipse)
        {
            // ─────────────────────────────────────────────────────────
            //  해석적 타원 궤도 계산
            //  (이전 코드와 동일)
            float M = M0 + n * elapsedTime;
            float E = M + e * Mathf.Sin(M) / (1f - Mathf.Sin(M + e) + Mathf.Sin(M));
            for (int i = 0; i < 8; i++)
            {
                float f = E - e * Mathf.Sin(E) - M;
                if (Mathf.Abs(f) < 1e-6f) break;
                float fPrime = 1f - e * Mathf.Cos(E);
                E -= f / fPrime;
            }

            float cosE = Mathf.Cos(E);
            float sinE = Mathf.Sin(E);
            float fac = Mathf.Sqrt((1f + e) / (1f - e));
            theta = 2f * Mathf.Atan(fac * sinE / (cosE + 1e-6f));
            if (cosE < 0f) theta += Mathf.PI;

            float r_val = a * (1f - e * cosE); // km 단위
            float angle = theta + omega;
            Vector3 newPos_m = new Vector3(
                r_val * Mathf.Cos(angle),
                r_val * Mathf.Sin(angle),
                0f
            );

            Vector3 finalPos = planetPos_m + newPos_m; // km 단위
            if (!float.IsNaN(finalPos.x))
            {
                shipInstance.transform.position = finalPos / unitScale; // Unity 좌표
                float speed_m = Mathf.Sqrt(mu * (2f / r_val - 1f / a)); // km/s
                float speedKm = speed_m;
                float thetaNorm = theta % (2f * Mathf.PI);
                if (thetaNorm < 0f) thetaNorm += 2f * Mathf.PI;
                float thetaDeg = thetaNorm * Mathf.Rad2Deg;
                OnLogUpdated?.Invoke($"Analytic θ={thetaDeg:F2}°, v={speedKm:0.00} km/s");
                OnSpeedUpdated?.Invoke(speedKm);
                Debug.Log($"[SimulationManager] Analytic: pos={finalPos}, speedKm={speedKm}, thetaDeg={thetaDeg}");
            }
            else
            {
                Debug.LogWarning("[SimulationManager] Analytic: Invalid finalPos=" + finalPos);
            }
            return;
        }

        // ─────────────────────────────────────────────────────────────
        // 3) 수치적 통합(RK4) – 고정된 작은 시간 스텝 적용
        //    scaledDt를 내부적으로 maxInternalStep(0.1s) 이하로 분할하여
        //    여러 번 RK4 적분 수행 → 안정성 향상
        float maxInternalStep = 0.1f; // (초) 내부 단위 시간 스텝 한계
        int nSteps = Mathf.CeilToInt(scaledDt / maxInternalStep);
        float internalDt = scaledDt / nSteps; // 진짜 RK4 한 스텝당 시간

        for (int i = 0; i < nSteps; i++)
        {
            // a) 현재 pos_m, vel_m 상태에 대해 RK4 1스텝 계산
            Vector3 rVec = planetPos_m - pos_m; // km 단위
            float distSqr = rVec.sqrMagnitude;

            Vector3 relativeVel = vel_m - planetVel; // km/s 단위
            float speed_m = relativeVel.magnitude;
            float speedKm = speed_m;

            if (distSqr > 1e-12f && !float.IsNaN(distSqr))
            {
                // k1
                Vector3 k1_v = CalculateAcceleration(pos_m, vel_m) * internalDt;
                Vector3 k1_r = vel_m * internalDt;

                // k2
                Vector3 pos2 = pos_m + 0.5f * k1_r;
                Vector3 vel2 = vel_m + 0.5f * k1_v;
                Vector3 k2_v = CalculateAcceleration(pos2, vel2) * internalDt;
                Vector3 k2_r = vel2 * internalDt;

                // k3
                Vector3 pos3 = pos_m + 0.5f * k2_r;
                Vector3 vel3 = vel_m + 0.5f * k2_v;
                Vector3 k3_v = CalculateAcceleration(pos3, vel3) * internalDt;
                Vector3 k3_r = vel3 * internalDt;

                // k4
                Vector3 pos4 = pos_m + k3_r;
                Vector3 vel4 = vel_m + k3_v;
                Vector3 k4_v = CalculateAcceleration(pos4, vel4) * internalDt;
                Vector3 k4_r = vel4 * internalDt;

                // 최종 합산
                vel_m += (k1_v + 2f * k2_v + 2f * k3_v + k4_v) / 6f;
                pos_m += (k1_r + 2f * k2_r + 2f * k3_r + k4_r) / 6f;

                Vector3 newPos = pos_m;
                if (!float.IsNaN(newPos.x))
                {
                    shipInstance.transform.position = newPos / unitScale; // Unity 좌표
                    shipInstance.transform.rotation = Quaternion.LookRotation(relativeVel, Vector3.up);

                    float ang = Mathf.Atan2(rVec.y, rVec.x) * Mathf.Rad2Deg;
                    if (ang < 0f) ang += 360f;

                    float dist = Mathf.Sqrt(distSqr);
                    OnLogUpdated?.Invoke($"Numeric θ={ang:F2}°, v={speedKm:0.00} km/s, dist={dist:F2} km");
                    OnSpeedUpdated?.Invoke(speedKm);
                    Debug.Log($"[SimulationManager] Numeric: pos={pos_m}, speedKm={speedKm}, theta={ang:F2}, dist={dist:F2} km");
                }
                else
                {
                    Debug.LogWarning("[SimulationManager] Numeric: Invalid newPos=" + newPos);
                }
            }
            else
            {
                OnLogUpdated?.Invoke($"Numeric: Invalid dist={(float)Math.Sqrt(distSqr):F2}, v={speed_m:0.00} km/s");
                OnSpeedUpdated?.Invoke(relativeVel.magnitude);
                Debug.LogWarning($"[SimulationManager] Numeric: Invalid distSqr={distSqr}, speedKm={relativeVel.magnitude}");
            }

            // ───────────── 행성 위치도 내부 스텝당 조금씩 업데이트 ─────────────
            // (scaledDt 중 planetVel에 의해 이동하는 양을 분할하여 반영)
            planetPos_m += planetVel * internalDt;
            planetTransform.position = planetPos_m / unitScale;
        }
    }

    private void ClearPrevious()
    {
        if (shipInstance != null)
        {
            Destroy(shipInstance);
            shipInstance = null;
            shipTrail = null;
            Debug.Log("[SimulationManager] ClearPrevious: 이전 우주선 제거");
        }
    }
}
