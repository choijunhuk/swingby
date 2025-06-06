using System;
using System.Collections;
using UnityEngine;
using UnityEngine.UIElements;

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
    public Vector3 shipPos = new Vector3(-80f, 0f, 0f);
    public Vector3 shipVel = new Vector3(2f, 1f, 0f);
    public float shipRadius = 0.01f;

    [Header("행성 파라미터 (UI 입력, 단위: km)")]
    public float planetMass = 100f;
    public Vector3 planetPos = Vector3.zero;
    public Vector3 planetVel = Vector3.zero;
    public float planetRadius = 0f;
    public bool isPointMass = true;

    [Header("물리 상수 & 설정")]
    public float gravConst = 1f;   // μ = gravConst * planetMass
    public float timeScale = 1f;   // 시뮬레이션 속도 배율
    public float trajWidth = 0.1f; // 트레일 굵기

    [Header("단위 스케일 (km → Unity 단위)")]
    public float unitScale = 1f;

    [Header("해석적 vs 수치 적분 방식")]
    [Tooltip("0: Analytic (Elliptic + Hyperbolic + Parabolic),\n1: RK4, 2: Euler")]
    public int simulationMethod = 0;

    [Header("Trail Colors")]
    public Color shipTrailColor = Color.cyan;
    public Color planetTrailColor = Color.yellow;

    [Header("카메라 모드")]
    public int camMode = 0;

    // UIManager와의 호환을 위해 이벤트 제공
    public event Action<string> OnLogUpdated;
    public event Action<float> OnSpeedUpdated;

    // ───────── 내부 상태 변수 ─────────
    private GameObject shipInstance;
    private Rigidbody shipRigidbody;
    private TrailRenderer shipTrail, planetTrail;

    private float mu;                    // μ = gravConst * planetMass
    private bool simulationRunning = false;
    private bool useAnalytic = false;    // Analytic 분기 사용 여부

    // 현재 위치·속도 (월드 좌표, km 단위)
    private Vector3 currentPosition;
    private Vector3 currentVelocity;

    private Vector3 planetPosition;      // 행성 위치 (km 단위)

    // Analytic 모드에서 사용하는 궤도 매개변수
    private double energy;               // 궤도 특정 에너지
    private double semiMajorAxis;        // a (타원: >0, 쌍곡선: <0)
    private double eccentricity;         // e
    private Vector3 orbitalPlaneNormal;  
    private Vector3 unitPerihelion;      // 궤도면 내 perihelion 방향 단위 벡터
    private Vector3 unitAxis2;           // 궤도면 내 두 번째 단위 벡터
    private double meanMotion;           // 평균운동 n = sqrt(μ/|a|^3)
    private double initialMeanAnomaly;   // M₀ (타원) 또는 M_{h0} (쌍곡선)
    private double simulationTime;       // 시뮬레이션 경과 시간

    void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
            mu = gravConst * planetMass;
            Debug.Log("[SimulationManager] Awake: Singleton 초기화 완료");
        }
        else
        {
            Destroy(gameObject);
            return;
        }
    }

    void Start()
    {
        // 행성 초기 위치 세팅 및 트레일 비활성화
        planetPosition = planetPos;
        if (planetTransform != null)
        {
            planetTransform.position = planetPosition / unitScale;
            if (!planetBody.TryGetComponent<TrailRenderer>(out planetTrail))
                planetTrail = planetBody.gameObject.AddComponent<TrailRenderer>();

            planetTrail.widthMultiplier    = trajWidth;
            planetTrail.time               = 100f;
            planetTrail.minVertexDistance  = 0.2f;
            planetTrail.startColor = planetTrail.endColor = planetTrailColor;
            planetTrail.Clear();
            planetTrail.emitting = false;
        }

        // 이미 Rigidbody가 붙어 있는 오브젝트가 있으면, 그것을 우주선으로 사용
        if (shipInstance == null && shipPrefab == null)
        {
            var existingRB = FindObjectOfType<Rigidbody>();
            if (existingRB != null)
            {
                shipInstance = existingRB.gameObject;
                shipRigidbody = existingRB;
            }
        }
    }

    void Update()
    {
        if (!simulationRunning) return;
        if (Mathf.Approximately(timeScale, 0f)) return;

        float dt = Time.deltaTime * timeScale;

        // 1) 행성 이동 (Numeric용, RK4/Euler 모두 해당)
        if (planetVel != Vector3.zero && dt > 0f)
        {
            planetPosition += planetVel * dt;
            if (planetTransform != null)
                planetTransform.position = planetPosition / unitScale;
        }

        // 2) 방식에 따라 분기
        if (useAnalytic)
        {
            UpdateAnalytic(dt);
        }
        else if (simulationMethod == 1)
        {
            UpdateNumericRK4(dt);
        }
        else if (simulationMethod == 2)
        {
            UpdateEuler(dt);
        }
    }

    /// <summary>
    /// UIManager가 "시작" 버튼을 눌렀을 때 호출됩니다.
    /// </summary>
    public void StartSimulation()
    {
        if (simulationRunning) return;
        StartCoroutine(StartSimulationRoutine());
    }

    /// <summary>
    /// UIManager가 "정지" 버튼을 눌렀을 때 호출됩니다.
    /// </summary>
    public void StopSimulation()
    {
        if (!simulationRunning) return;
        simulationRunning = false;

        if (shipRigidbody != null)
        {
            shipRigidbody.velocity        = Vector3.zero;
            shipRigidbody.angularVelocity = Vector3.zero;
            shipRigidbody.isKinematic     = true;
        }
        if (shipTrail != null)   shipTrail.emitting   = false;
        if (planetTrail != null) planetTrail.emitting = false;

        Debug.Log("[SimulationManager] 시뮬레이션 정지");
        OnLogUpdated?.Invoke("--- 시뮬레이션 정지 ---");
    }

    /// <summary>
    /// UIManager가 "재개" 버튼을 눌렀을 때 호출됩니다.
    /// </summary>
    public void ResumeSimulation()
    {
        if (simulationRunning) return;
        simulationRunning = true;
        if (shipRigidbody != null) shipRigidbody.isKinematic = true;
        if (shipTrail != null)     shipTrail.emitting   = true;
        if (planetTrail != null)   planetTrail.emitting = true;
        Debug.Log("[SimulationManager] 시뮬레이션 재개됨");
        OnLogUpdated?.Invoke("--- 시뮬레이션 재개 ---");
    }

    /// <summary>
    /// 실제로 우주선·행성을 씬에 배치하고, Analytic/RK4/Euler 초기화를 마친 뒤 시뮬레이션을 시작합니다.
    /// </summary>
    private IEnumerator StartSimulationRoutine()
    {
        // 1) 이전 우주선 있으면 삭제
        if (shipInstance != null)
        {
            Destroy(shipInstance);
            shipInstance  = null;
            shipRigidbody = null;
            shipTrail     = null;
            Debug.Log("[SimulationManager] ClearPrevious: 이전 우주선 제거");
        }

        // 2) μ 재계산
        mu = gravConst * planetMass;

        // 3) 행성 초기 위치 및 트레일 활성화
        planetPosition = planetPos;
        if (planetTransform != null)
        {
            planetTransform.position = planetPosition / unitScale;
            planetTrail.widthMultiplier    = trajWidth;
            planetTrail.time               = 100f;
            planetTrail.minVertexDistance  = 0.2f;
            planetTrail.startColor = planetTrail.endColor = planetTrailColor;
            planetTrail.Clear();
            planetTrail.emitting = true;
        }

        // 4) 우주선 생성 또는 재사용
        currentPosition = shipPos;
        currentVelocity = shipVel;
        Vector3 shipUnityPos = currentPosition / unitScale;

        if (shipInstance == null)
        {
            shipInstance = Instantiate(shipPrefab, shipUnityPos, Quaternion.identity);
            if (shipInstance == null)
            {
                Debug.LogError("[SimulationManager] 우주선 생성 실패!");
                yield break;
            }
        }
        else
        {
            shipInstance.transform.position = shipUnityPos;
        }

        shipRigidbody = shipInstance.GetComponent<Rigidbody>();
        if (shipRigidbody != null)
        {
            shipRigidbody.position        = shipUnityPos;
            shipRigidbody.rotation        = Quaternion.identity;
            shipRigidbody.velocity        = Vector3.zero;
            shipRigidbody.angularVelocity = Vector3.zero;
            shipRigidbody.mass            = shipMass;
            shipRigidbody.useGravity      = false;
            shipRigidbody.isKinematic     = true;
        }

        Debug.Log($"[SimulationManager] 우주선 생성: shipPos={shipPos}, shipVel={shipVel}");
        if (!shipInstance.TryGetComponent<TrailRenderer>(out shipTrail))
            shipTrail = shipInstance.AddComponent<TrailRenderer>();

        shipTrail.widthMultiplier    = trajWidth;
        shipTrail.time               = 100f;
        shipTrail.minVertexDistance  = 0.2f;
        shipTrail.startColor = shipTrail.endColor = shipTrailColor;
        shipTrail.Clear();
        shipTrail.emitting = true;

        // 5) 궤도 에너지 계산 (현재 위치·속도로부터)
        {
            double rx    = currentPosition.x;
            double ry    = currentPosition.y;
            double rz    = currentPosition.z;
            double vx    = currentVelocity.x;
            double vy    = currentVelocity.y;
            double vz    = currentVelocity.z;
            double r0Sqr = rx * rx + ry * ry + rz * rz;
            double r0Mag = Math.Sqrt(r0Sqr);

            double mu_d  = mu;
            double v0Sqr = vx * vx + vy * vy + vz * vz;
            energy       = 0.5 * v0Sqr - mu_d / r0Mag;
        }

        // 6) 방식에 따라 초기화 분기
        if (simulationMethod == 0)
        {
            // Analytic 모드 선택 → energy<0(타원)만 허용
            if (energy < 0.0)
            {
                // ——— Analytic Elliptic 초기화 ———
                Vector3 hVec = Vector3.Cross(currentPosition, currentVelocity);
                orbitalPlaneNormal = hVec.normalized;
                Vector3 vCrossH = Vector3.Cross(currentVelocity, hVec);
                Vector3 eVec = (vCrossH / (float)mu) - (currentPosition.normalized);
                eccentricity = eVec.magnitude;

                semiMajorAxis = -mu / (2.0 * energy);

                // Perihelion 단위 벡터
                unitPerihelion = (eccentricity < 1e-6)
                    ? currentPosition.normalized
                    : eVec.normalized;
                unitAxis2 = Vector3.Cross(orbitalPlaneNormal, unitPerihelion).normalized;

                // 초진각 ν₀
                double cosNu0 = Vector3.Dot(currentPosition.normalized, unitPerihelion);
                cosNu0 = Math.Clamp(cosNu0, -1.0, 1.0);
                double dot_r_e = currentPosition.x * eVec.x
                               + currentPosition.y * eVec.y
                               + currentPosition.z * eVec.z;
                double nu0 = (dot_r_e >= 0.0)
                    ? Math.Acos(cosNu0)
                    : 2.0 * Math.PI - Math.Acos(cosNu0);

                // Eccentric anomaly E₀
                double cosE0 = (eccentricity + Math.Cos(nu0)) / (1 + eccentricity * Math.Cos(nu0));
                cosE0 = Math.Clamp(cosE0, -1.0, 1.0);
                double sinE0 = Math.Sqrt(Math.Max(0.0, 1 - cosE0 * cosE0));
                if (nu0 > Math.PI) sinE0 = -sinE0;
                double E0 = Math.Atan2(sinE0, cosE0);
                if (E0 < 0) E0 += 2.0 * Math.PI;

                // 평균 이상 M₀
                initialMeanAnomaly = E0 - eccentricity * sinE0;

                // 평균운동 n
                meanMotion = Math.Sqrt(mu / Math.Pow(semiMajorAxis, 3));

                useAnalytic = true;
            }
            else
            {
                // 에너지 ≥ 0(포물선 또는 쌍곡선) → Analytic 불가
                string msg = $"[SimulationManager] 오류: Analytic 모드는 타원(energy<0)만 지원합니다. 현재 energy={energy:F3}";
                Debug.LogError(msg);
                OnLogUpdated?.Invoke($"--- 오류: Analytic 모드는 타원만 지원합니다 (현재 energy={energy:F3}) ---");
                yield break;
            }

            OnLogUpdated?.Invoke($"--- 시뮬레이션 시작 (Analytic Elliptic) ---");
        }
        else if (simulationMethod == 1)
        {
            // RK4 (수치 통합) 초기화 없음
            useAnalytic = false;
            OnLogUpdated?.Invoke($"--- 시뮬레이션 시작 (Numeric RK4) ---");
        }
        else if (simulationMethod == 2)
        {
            // Euler 초기화 없음 (단순 오일러 방식)
            useAnalytic = false;
            OnLogUpdated?.Invoke($"--- 시뮬레이션 시작 (Euler) ---");
        }

        // 시뮬레이션 시간 초기화 및 시작
        simulationTime = 0.0;
        simulationRunning = true;
        Debug.Log($"[SimulationManager] 시뮬레이션 시작 (Method={(simulationMethod == 0 ? "Analytic" : simulationMethod == 1 ? "RK4" : "Euler")})");

        yield return null;
    }

   /// <summary>
/// Analytic(해석적) 업데이트: 타원(energy<0)만 처리
/// </summary>
/// <summary>
/// Analytic(해석적) 업데이트: 타원(energy<0)만 처리
/// </summary>
/// <summary>
/// Analytic(해석적) 업데이트: 타원(energy<0)만 처리
/// </summary>
private void UpdateAnalytic(double dt)
{
    simulationTime += dt;

    // 1) 평균근 M 계산
    double M = initialMeanAnomaly + meanMotion * simulationTime;
    M %= (2.0 * Math.PI);

    // 2) Kepler 방정식(Newton-Raphson)으로 이심근 E 찾기
    double E = M;
    for (int i = 0; i < 10; i++)
    {
        double f  = E - eccentricity * Math.Sin(E) - M;
        double f1 = 1.0 - eccentricity * Math.Cos(E);
        double dE = f / f1;
        E -= dE;
        if (Math.Abs(dE) < 1e-7) break;
    }

    // 3) 궤도 반경 r, factor 계산
    double cosE = Math.Cos(E);
    double sinE = Math.Sin(E);
    double r_val = semiMajorAxis * (1.0 - eccentricity * cosE);
    double factor = Math.Sqrt(Math.Max(0.0, 1.0 - eccentricity * eccentricity));

    // 4) 궤도면 내 2D 좌표 (x_op, y_op)
    double x_op = semiMajorAxis * (cosE - eccentricity);
    double y_op = semiMajorAxis * factor * sinE;

    // 5) 3D 공간(월드 좌표)으로 변환
    Vector3 posPlane = (float)x_op * unitPerihelion + (float)y_op * unitAxis2;
    currentPosition = posPlane;
    if (shipInstance != null)
    {
        shipInstance.transform.position = currentPosition / unitScale;
    }

    // 6) 속도 벡터 계산 (world‐space)
    double denom = 1.0 - eccentricity * cosE;
    if (Math.Abs(denom) < 1e-8) denom = denom >= 0 ? 1e-8 : -1e-8;
    double E_dot = meanMotion / denom;

    double vx_p = -semiMajorAxis * sinE * E_dot;
    double vy_p =  semiMajorAxis * factor * cosE * E_dot;
    Vector3 velPlane = (float)vx_p * unitPerihelion + (float)vy_p * unitAxis2;
    currentVelocity = velPlane;

    double speed_m = velPlane.magnitude;

    // ★ world‐space 속도(currentVelocity) 기준으로 각도 계산
    double thetaDeg = Math.Atan2(currentVelocity.y, currentVelocity.x) * Mathf.Rad2Deg;
    if (thetaDeg < 0) thetaDeg += 360.0;

    OnLogUpdated?.Invoke($"Analytic (Ellipse) θ={thetaDeg:F2}°, v={speed_m:0.00} km/s");
    OnSpeedUpdated?.Invoke((float)speed_m);
}





    /// <summary>
    /// Numeric RK4 방식으로 궤도를 통합합니다.
    /// </summary>
    private void UpdateNumericRK4(float dt)
{
    const int NUM_SUBSTEPS = 1;
    float subDt = dt / NUM_SUBSTEPS;

    for (int i = 0; i < NUM_SUBSTEPS; i++)
    {
        // --- k1 단계 ---
        Vector3 a0   = ComputeAcceleration(currentPosition);
        Vector3 k1_p = currentVelocity;
        Vector3 k1_v = a0;

        // --- k2 단계 ---
        Vector3 p_mid1 = currentPosition + k1_p * (subDt * 0.5f);
        Vector3 v_mid1 = currentVelocity + k1_v * (subDt * 0.5f);
        Vector3 a_mid1 = ComputeAcceleration(p_mid1);
        Vector3 k2_p   = v_mid1;
        Vector3 k2_v   = a_mid1;

        // --- k3 단계 ---
        Vector3 p_mid2 = currentPosition + k2_p * (subDt * 0.5f);
        Vector3 v_mid2 = currentVelocity + k2_v * (subDt * 0.5f);
        Vector3 a_mid2 = ComputeAcceleration(p_mid2);
        Vector3 k3_p   = v_mid2;
        Vector3 k3_v   = a_mid2;

        // --- k4 단계 ---
        Vector3 p_end  = currentPosition + k3_p * subDt;
        Vector3 v_end  = currentVelocity + k3_v * subDt;
        Vector3 a_end  = ComputeAcceleration(p_end);
        Vector3 k4_p   = v_end;
        Vector3 k4_v   = a_end;

        // --- RK4 공식 적용 ---
        currentPosition += (subDt / 6f) * (k1_p + 2f * k2_p + 2f * k3_p + k4_p);
        currentVelocity += (subDt / 6f) * (k1_v + 2f * k2_v + 2f * k3_v + k4_v);
    }

    // 한 프레임(= dt 전체) 통합이 끝난 뒤, 실제 오브젝트 위치 업데이트
    if (shipInstance != null)
        shipInstance.transform.position = currentPosition / unitScale;

    // 로그용 속도·거리 등 업데이트
    float speed_m = currentVelocity.magnitude;
    Vector3 rVec = (planetPosition / unitScale) - (currentPosition / unitScale);
    float thetaDeg = Mathf.Atan2(rVec.y, rVec.x) * Mathf.Rad2Deg;
    if (thetaDeg < 0) thetaDeg += 360f;
    float dist = rVec.magnitude * unitScale;
    OnLogUpdated?.Invoke($"RK4 θ={thetaDeg:F2}°, v={speed_m:0.00} km/s, dist={dist:F2} km");
    OnSpeedUpdated?.Invoke(speed_m);
}


    /// <summary>
    /// 단순 오일러(Euler) 방식으로 궤도를 통합합니다.
    /// </summary>
    /// <summary>
/// 단순 오일러(Euler) 방식으로 궤도를 통합합니다.
/// → dt를 100등분(subDt)해서 100번 반복하면, 한 프레임 전체가 dt가 유지됩니다.
/// </summary>
/// <summary>
/// 단순 오일러(Euler) 방식으로 궤도를 통합합니다.
/// → dt를 100등분(subDt)해서 100번 반복하면, 한 프레임 전체가 dt가 유지됩니다.
/// </summary>
private void UpdateEuler(float dt)
{
    const int NUM_SUBSTEPS = 1;
    float subDt = dt / NUM_SUBSTEPS;

    for (int i = 0; i < NUM_SUBSTEPS; i++)
    {
        // (1) 가속도 계산
        Vector3 a = ComputeAcceleration(currentPosition);

        // (2) 속도와 위치를 subDt만큼 Forward Euler로 업데이트
        currentVelocity += a * subDt;
        currentPosition += currentVelocity * subDt;
    }

    // 한 프레임(= dt 전체) 통합이 끝난 뒤, 실제 오브젝트 위치 업데이트
    if (shipInstance != null)
        shipInstance.transform.position = currentPosition / unitScale;

    // 로그용 속도·거리·진두각 업데이트
    float speed_m = currentVelocity.magnitude;
    Vector3 rVec = (planetPosition / unitScale) - (currentPosition / unitScale);
    float thetaDeg = Mathf.Atan2(rVec.y, rVec.x) * Mathf.Rad2Deg;
    if (thetaDeg < 0) thetaDeg += 360f;
    float dist = rVec.magnitude * unitScale;
    OnLogUpdated?.Invoke($"Euler θ={thetaDeg:F2}°, v={speed_m:0.00} km/s, dist={dist:F2} km");
    OnSpeedUpdated?.Invoke(speed_m);
}



    /// <summary>
    /// 중력 가속도 계산: a = - μ·r / |r|^3
    /// </summary>
    private Vector3 ComputeAcceleration(Vector3 pos)
    {
        float rSqr = pos.sqrMagnitude;
        if (rSqr < 1e-12f) return Vector3.zero;
        float invR = 1f / Mathf.Sqrt(rSqr);
        float invR2 = invR * invR;
        return -mu * pos * (invR2 * invR);
    }

    // UIManager와의 호환을 위한 프로퍼티
    public bool IsRunning => simulationRunning;
    public Transform ShipTransform => shipInstance != null ? shipInstance.transform : null;
    public Transform PlanetTransform => planetTransform;
}
