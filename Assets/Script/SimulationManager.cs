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
    public float gravConst = 6.674e-11f;    // (m^3/(kg s^2))
    public float timeScale = 3.0f;
    public float trajWidth = 0.1f;

    [Header("단위 스케일 (km → Unity 단위)")]
    [Tooltip("몇으로 나눠서 Unity 좌표(u)로 변환할지 입력하세요. 예: 100 (1유닛=100km), 1000 (1유닛=1000km)")]
    public float unitScale = 100f;

    [Header("해석적 vs 수치 적분 방식")]
    [Tooltip("0: 해석적(Analytic), 1: 수치(Euler)")]
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

    // 두 체 문제 파라미터
    private float mu;       // μ = G*(m_planet + m_ship) ≈ G*m_planet (단위: km^3/s^2)
    private float a;        // 준장반경(semi-major axis)
    private float e;        // 이심률(eccentricity)
    private float omega;    // 궤도 고도각(argument of periapsis)
    private float theta0;   // 초기 진두각(true anomaly at t=0)
    private float M0;       // 초기 평균근(mean anomaly at t=0)
    private float n;        // 평균 운동(mean motion)
    private Vector3 r0;     // 초기 상대위치 (shipPos - planetPos) [km]
    private Vector3 v0;     // 초기 상대속도 (shipVel - planetVel) [km/s]
    private float r0Mag;    // 초기 상대거리 |r0|
    private float energy;   // 특유 에너지 specific energy = v0^2/2 - μ/r0

    private float h;        // 궤도 각운동량 크기 |r0 × v0|
    private float p;        // 이심점 매개변수, p = h^2/μ

    private float elapsedTime;
    private bool running;
    private bool useAnalytic;  // true: Analytic, false: Numeric(Euler)

    // 궤도 종류 구분
    private enum OrbitType { Elliptic, Parabolic, Hyperbolic }
    private OrbitType orbitType;

    // 시뮬레이션 내부 상태 (km 단위)
    private Vector3 pos_m;       // 우주선 현재 위치 [km]
    private Vector3 vel_m;       // 우주선 현재 속도 [km/s]
    private Vector3 planetPos_m; // 행성 현재 위치 [km]

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

        // 1) μ 계산 (km^3/s^2)
        mu = gravConst * planetMass * 1e-9f;
        //   (원래 G [m^3/(kg s^2)] * M[kg] * (1/1000)^3 => km^3/s^2)

        // 2) 초기 행성 위치 셋업
        planetPos_m = planetPos;                       // [km]
        planetTransform.position = planetPos_m / unitScale;  // Unity 좌표
        planetTransform.localScale = Vector3.one;
        Debug.Log($"[SimulationManager] StartSimulation: planetPos_m={planetPos_m}, unitScale={unitScale}");

        // 트레일 초기화 (planet)
        if (!planetBody.TryGetComponent<TrailRenderer>(out planetTrail))
            planetTrail = planetBody.gameObject.AddComponent<TrailRenderer>();
        planetTrail.widthMultiplier = trajWidth;
        planetTrail.time = 100f;
        planetTrail.minVertexDistance = 0.2f;
        planetTrail.startColor = planetTrail.endColor = planetTrailColor;
        planetTrail.Clear();
        planetTrail.emitting = true;

        // 3) 우주선 생성 및 초기 설정
        shipInstance = Instantiate(shipPrefab);
        if (shipInstance == null) Debug.LogError("[SimulationManager] 우주선 생성 실패!");
        shipInstance.transform.localScale = Vector3.one;
        pos_m = shipPos;                                // [km]
        shipInstance.transform.position = pos_m / unitScale; // Unity 좌표
        vel_m = shipVel;                                // [km/s]
        Debug.Log($"[SimulationManager] 우주선 생성: shipPos={shipPos}, shipVel={shipVel}");

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

        // 4) Analytic vs Numeric(Euler) 선택
        useAnalytic = (simulationMethod == 0);

        // 두 체 상대위치/속도
        r0 = shipPos - planetPos_m;    // [km]
        v0 = shipVel - planetVel;      // [km/s]
        r0Mag = r0.magnitude;

        float v0sq = v0.sqrMagnitude;
        energy = 0.5f * v0sq - mu / r0Mag; // Specific Energy

        // 각운동량 h
        h = Vector3.Cross(r0, v0).magnitude;
        // 궤도 매개변수 p
        if (Mathf.Abs(mu) > 0f)
            p = (h * h) / mu;
        else
            p = 0f;

        // 5) 궤도 타입 결정 (Elliptic / Parabolic / Hyperbolic)
        if (energy < 0f)
        {
            orbitType = OrbitType.Elliptic;
        }
        else if (Mathf.Abs(energy) < 1e-8f)
        {
            orbitType = OrbitType.Parabolic;
        }
        else
        {
            orbitType = OrbitType.Hyperbolic;
        }

        // 6) Analytic 모드일 때, 궤도 파라미터 미리 계산 (M0, n, a, e, omega, θ0 등)
        if (useAnalytic)
        {
            switch (orbitType)
            {
                case OrbitType.Elliptic:
                    {
                        // (1) 이심률 벡터 계산
                        Vector3 eVec = ((v0sq - mu / r0Mag) * r0 - Vector3.Dot(r0, v0) * v0) / mu;
                        e = eVec.magnitude;

                        // (2) 궤도 방향 (argument of periapsis)
                        float rawOmega = Mathf.Atan2(eVec.y, eVec.x);
                        if (rawOmega < 0f) rawOmega += 2f * Mathf.PI;
                        omega = rawOmega;

                        // (3) 준장반경 a (에너지 < 0)
                        a = -mu / (2f * energy);

                        // (4) 평균 운동 n
                        n = Mathf.Sqrt(mu / (a * a * a));

                        // (5) 초기 진두각 θ0
                        float cosTheta0 = Vector3.Dot(eVec, r0) / (e * r0Mag);
                        cosTheta0 = Mathf.Clamp(cosTheta0, -1f, 1f);
                        theta0 = Mathf.Acos(cosTheta0);
                        if (Vector3.Dot(r0, v0) < 0f)
                            theta0 = 2f * Mathf.PI - theta0;

                        // (6) 초기 이심근 E0 (Elliptic Anomaly)
                        float tanHalfTheta0 = Mathf.Tan(theta0 / 2f);
                        float sqrtFactor = Mathf.Sqrt((1f - e) / (1f + e));
                        float E0 = 2f * Mathf.Atan(sqrtFactor * tanHalfTheta0);
                        if (E0 < 0f) E0 += 2f * Mathf.PI;

                        // (7) 초기 평균근 M0
                        M0 = E0 - e * Mathf.Sin(E0);

                        Debug.Log($"[SimulationManager] Analytic Elliptic 초기값: a={a}, e={e}, omega={omega * Mathf.Rad2Deg}°, θ0={theta0 * Mathf.Rad2Deg}°, M0={M0}");
                        break;
                    }

                case OrbitType.Parabolic:
                    {
                        // p = h^2/μ (이미 위에서 계산해 둠)

                        // 초기 진두각 ν0
                        float cosNu0 = Vector3.Dot(r0, v0) / (r0Mag * v0.magnitude);
                        cosNu0 = Mathf.Clamp(cosNu0, -1f, 1f);
                        float nu0 = Mathf.Acos(cosNu0);
                        if (Vector3.Dot(r0, v0) < 0f)
                            nu0 = 2f * Mathf.PI - nu0;
                        theta0 = nu0;

                        // Parabolic Anomaly D0
                        float D0 = Mathf.Tan(theta0 / 2f);

                        // Barker’s equation 상수 형태
                        float factor = Mathf.Sqrt(p * p * p / mu);
                        float t0 = 0.5f * factor * (D0 + (D0 * D0 * D0) / 3f);
                        elapsedTime = -t0;

                        Debug.Log($"[SimulationManager] Analytic Parabolic 초기값: p={p}, θ0={theta0 * Mathf.Rad2Deg}°, t0={t0}");
                        break;
                    }

                case OrbitType.Hyperbolic:
                    {
                        // 이심률 벡터 계산
                        Vector3 eVec = ((v0sq - mu / r0Mag) * r0 - Vector3.Dot(r0, v0) * v0) / mu;
                        e = eVec.magnitude;

                        // 궤도 방향
                        float rawOmega = Mathf.Atan2(eVec.y, eVec.x);
                        if (rawOmega < 0f) rawOmega += 2f * Mathf.PI;
                        omega = rawOmega;

                        // 준장반경 a (< 0)
                        a = -mu / (2f * energy);

                        // 평균 운동 (hyperbolic)
                        n = Mathf.Sqrt(mu / (-(a * a * a)));

                        // 초기 진두각 ν0
                        float cosNu0_h = Vector3.Dot(r0, v0) / (r0Mag * v0.magnitude);
                        cosNu0_h = Mathf.Clamp(cosNu0_h, -1f, 1f);
                        float nu0_h = Mathf.Acos(cosNu0_h);
                        if (Vector3.Dot(r0, v0) < 0f)
                            nu0_h = 2f * Mathf.PI - nu0_h;
                        theta0 = nu0_h;

                        // Hyperbolic Anomaly F0
                        float tanhHalfF0 = Mathf.Sqrt((e - 1f) / (e + 1f)) * Mathf.Tan(theta0 / 2f);
                        float F0 = 2f * AtanhSafe(tanhHalfF0);
                        M0 = e * Sinh(F0) - F0;

                        Debug.Log($"[SimulationManager] Analytic Hyperbolic 초기값: a={a}, e={e}, omega={omega * Mathf.Rad2Deg}°, θ0={theta0 * Mathf.Rad2Deg}°, F0={F0}, M0={M0}");
                        break;
                    }
            }

            running = true;
            OnLogUpdated?.Invoke($"--- 시뮬레이션 시작 (해석적 모드 / {orbitType.ToString()}) ---");
            Debug.Log($"[SimulationManager] 시뮬레이션 시작 (Analytic, {orbitType})");
        }
        else
        {
            // Numeric(Euler) 모드만 사용
            running = true;
            OnLogUpdated?.Invoke("--- 시뮬레이션 시작 (수치 모드: Euler) ---");
            Debug.Log("[SimulationManager] 시뮬레이션 시작 (Numeric: Euler)");
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
        Vector3 rVec = planetPos_m - position; // km
        float distSqr = rVec.sqrMagnitude;
        if (distSqr > 1e-12f && !float.IsNaN(distSqr))
        {
            float dist = Mathf.Sqrt(distSqr);
            return (mu / (dist * dist)) * rVec.normalized; // km/s^2
        }
        Debug.LogWarning("[SimulationManager] CalculateAcceleration: Invalid distSqr=" + distSqr);
        return Vector3.zero;
    }

    void FixedUpdate()
    {
        if (!running || shipInstance == null) return;

        float dt = Time.fixedDeltaTime * timeScale;
        elapsedTime += dt;

        // 1) 행성 위치 업데이트
        planetPos_m += planetVel * dt; // km
        planetTransform.position = planetPos_m / unitScale; // Unity 좌표
        Debug.Log($"[SimulationManager] FixedUpdate: planetPos_m={planetPos_m}, dt={dt}");

        // 2) Analytic vs Numeric 분기
        if (useAnalytic)
        {
            switch (orbitType)
            {
                case OrbitType.Elliptic:
                    UpdateElliptic();
                    break;

                case OrbitType.Parabolic:
                    UpdateParabolic();
                    break;

                case OrbitType.Hyperbolic:
                    UpdateHyperbolic();
                    break;
            }
        }
        else
        {
            UpdateEuler(dt);  // ★ RK4 → Euler 로 교체
        }
    }

    #region ■ Analytic Orbit 계산 ■

    /// <summary>
    /// 타원 궤도(E < 0) 해석적 계산
    /// </summary>
    private void UpdateElliptic()
    {
        // M = M0 + n * t
        float M = M0 + n * elapsedTime;

        // Kepler's Equation (타원용): M = E - e sin E
        float E = M;
        for (int i = 0; i < 8; i++)
        {
            float f = E - e * Mathf.Sin(E) - M;
            if (Mathf.Abs(f) < 1e-6f) break;
            float fPrime = 1f - e * Mathf.Cos(E);
            E -= f / fPrime;
        }

        // true anomaly θ 계산
        float cosE = Mathf.Cos(E);
        float sinE = Mathf.Sin(E);
        float fac = Mathf.Sqrt((1f + e) / (1f - e));
        float theta = 2f * Mathf.Atan(fac * sinE / (cosE + 1e-6f));
        if (cosE < 0f) theta += Mathf.PI;

        // 궤도 반경 r = a * (1 - e cos E)
        float r_val = a * (1f - e * cosE);

        // 행성 기준 위치 newPos_m
        Vector3 newPos_m = new Vector3(
            r_val * Mathf.Cos(theta + omega),
            r_val * Mathf.Sin(theta + omega),
            0f
        );

        // 최종 좌표: 행성 위치 + 상대위치
        Vector3 finalPos = planetPos_m + newPos_m; // [km]
        if (!float.IsNaN(finalPos.x))
        {
            shipInstance.transform.position = finalPos / unitScale; // Unity 좌표
            float speed_m = Mathf.Sqrt(mu * (2f / r_val - 1f / a)); // 속도 크기 [km/s]
            float thetaDeg = (theta % (2f * Mathf.PI)) * Mathf.Rad2Deg;
            if (thetaDeg < 0f) thetaDeg += 360f;

            OnLogUpdated?.Invoke($"Analytic Elliptic θ={thetaDeg:F2}°, v={speed_m:0.00} km/s");
            OnSpeedUpdated?.Invoke(speed_m);
            Debug.Log($"[SimulationManager] Analytic Elliptic: pos={finalPos}, v={speed_m}, θ={thetaDeg}");
        }
        else
        {
            Debug.LogWarning("[SimulationManager] Analytic Elliptic: Invalid finalPos=" + finalPos);
        }
    }

    /// <summary>
    /// 포물선 궤도(E ≈ 0) 해석적 계산 (Barker’s Equation)
    /// </summary>
    private void UpdateParabolic()
    {
        float t_rel = elapsedTime; // elapsedTime = t - t0 상태

        // Parabolic Anomaly D: Barker’s equation D + D^3/3 = 2 * sqrt(μ/p^3) * (t - t0)
        float C = 2f * Mathf.Sqrt(mu / (p * p * p)) * t_rel;

        // Newton–Raphson 으로 D 풀기
        float D = C;
        for (int i = 0; i < 8; i++)
        {
            float f = D + (D * D * D) / 3f - C;
            float fPrime = 1f + D * D;
            D -= f / fPrime;
        }

        // true anomaly ν: ν = 2 arctan(D)
        float nu = 2f * Mathf.Atan(D);

        // 궤도 반경 r = p / (1 + cos ν)
        float r_val = p / (1f + Mathf.Cos(nu));

        // 궤도 각도 θ_total = ν + omega
        float angle = nu + omega;

        Vector3 newPos_m = new Vector3(
            r_val * Mathf.Cos(angle),
            r_val * Mathf.Sin(angle),
            0f
        );

        Vector3 finalPos = planetPos_m + newPos_m; // [km]
        if (!float.IsNaN(finalPos.x))
        {
            shipInstance.transform.position = finalPos / unitScale;
            float speed_m = Mathf.Sqrt(2f * mu / r_val);
            float nuDeg = (nu * Mathf.Rad2Deg) % 360f;
            if (nuDeg < 0f) nuDeg += 360f;

            OnLogUpdated?.Invoke($"Analytic Parabolic ν={nuDeg:F2}°, v={speed_m:0.00} km/s, r={r_val:0.00} km");
            OnSpeedUpdated?.Invoke(speed_m);
            Debug.Log($"[SimulationManager] Analytic Parabolic: pos={finalPos}, v={speed_m}, ν={nuDeg}, r={r_val}");
        }
        else
        {
            Debug.LogWarning("[SimulationManager] Analytic Parabolic: Invalid finalPos=" + finalPos);
        }
    }

    /// <summary>
    /// 쌍곡선 궤도(E > 0) 해석적 계산 (Hyperbolic Anomaly 사용)
    /// </summary>
    private void UpdateHyperbolic()
    {
        float t_rel = elapsedTime;

        // 평균근 M = M0 + n * t
        float M = M0 + n * t_rel;

        // Newton–Raphson 으로 F 풀기 (Hyperbolic Kepler)
        float F = Mathf.Log(2f * M / e + 1.8f);
        for (int i = 0; i < 8; i++)
        {
            float sinhF = Sinh(F);
            float coshF = Cosh(F);
            float fEq = e * sinhF - F - M;
            if (Mathf.Abs(fEq) < 1e-6f) break;
            float dfEq = e * coshF - 1f;
            F -= fEq / dfEq;
        }

        // true anomaly ν
        float tanhHalfF = Sinh(F) / (Cosh(F) + 1f); // tanh(F/2)
        float sqrtFactor = Mathf.Sqrt((e + 1f) / (e - 1f));
        float nu = 2f * Mathf.Atan(sqrtFactor * tanhHalfF);
        if (nu < 0f) nu += 2f * Mathf.PI;

        // 궤도 반경 r = a (e cosh F - 1)
        float r_val = a * (e * Cosh(F) - 1f);

        float angle = nu + omega;
        Vector3 newPos_m = new Vector3(
            r_val * Mathf.Cos(angle),
            r_val * Mathf.Sin(angle),
            0f
        );

        Vector3 finalPos = planetPos_m + newPos_m; // [km]
        if (!float.IsNaN(finalPos.x))
        {
            shipInstance.transform.position = finalPos / unitScale;
            float speed_m = Mathf.Sqrt(mu * (2f / r_val + 1f / (-a)));
            float nuDeg = (nu * Mathf.Rad2Deg) % 360f;
            if (nuDeg < 0f) nuDeg += 360f;

            OnLogUpdated?.Invoke($"Analytic Hyperbolic ν={nuDeg:F2}°, v={speed_m:0.00} km/s, r={r_val:0.00} km");
            OnSpeedUpdated?.Invoke(speed_m);
            Debug.Log($"[SimulationManager] Analytic Hyperbolic: pos={finalPos}, v={speed_m}, ν={nuDeg}, r={r_val}");
        }
        else
        {
            Debug.LogWarning("[SimulationManager] Analytic Hyperbolic: Invalid finalPos=" + finalPos);
        }
    }

    #endregion

    #region ■ Numeric (Euler) Orbit 계산 ■

    /// <summary>
    /// Numeric 모드: Explicit Euler 적분
    /// </summary>
    private void UpdateEuler(float dt)
    {
        // (1) 중력 가속도 계산 (현재 위치 기준)
        Vector3 accel = CalculateAcceleration(pos_m, vel_m); // km/s^2

        // (2) Explicit Euler: 속도와 위치를 각각 dt만큼 한 단계 앞으로 진행
        vel_m += accel * dt;            // v_{n+1} = v_n + a_n * dt
        pos_m += vel_m * dt;            // r_{n+1} = r_n + v_{n+1} * dt  (또는 v_n * dt)

        Vector3 newPos = pos_m;
        Vector3 relativeVel = vel_m - planetVel;
        float speed_m = relativeVel.magnitude; // km/s

        Vector3 rVec = planetPos_m - pos_m;
        float dist = Mathf.Sqrt(rVec.sqrMagnitude);

        if (!float.IsNaN(newPos.x))
        {
            shipInstance.transform.position = newPos / unitScale; // Unity 좌표
            shipInstance.transform.rotation = Quaternion.LookRotation(relativeVel, Vector3.up);
            float ang = Mathf.Atan2(rVec.y, rVec.x) * Mathf.Rad2Deg;
            if (ang < 0f) ang += 360f;

            OnLogUpdated?.Invoke($"Euler θ={ang:F2}°, v={speed_m:0.00} km/s, dist={dist:F2} km");
            OnSpeedUpdated?.Invoke(speed_m);
            Debug.Log($"[SimulationManager] Euler: pos={pos_m}, speed={speed_m:0.00}, θ={ang:F2}, dist={dist:F2} km");
        }
        else
        {
            Debug.LogWarning("[SimulationManager] Euler: Invalid newPos=" + newPos);
            OnLogUpdated?.Invoke($"Euler: Invalid 상태, v={speed_m:0.00} km/s");
            OnSpeedUpdated?.Invoke(speed_m);
        }
    }

    #endregion

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

    #region ■ 수학 보조 함수 ■

    /// <summary>
    /// 안전한 atanh(x) 계산: x가 ±1에 근접할 때 NaN 방지
    /// </summary>
    private float AtanhSafe(float x)
    {
        x = Mathf.Clamp(x, -0.999999f, 0.999999f);
        return 0.5f * Mathf.Log((1f + x) / (1f - x));
    }

    /// <summary>
    /// 쌍곡선(sinh) 계산 (C#에서 기본 제공하지 않으므로 직접)
    /// </summary>
    private float Sinh(float x)
    {
        float ex = Mathf.Exp(x), enx = Mathf.Exp(-x);
        return 0.5f * (ex - enx);
    }

    /// <summary>
    /// 쌍곡선(cosh) 계산
    /// </summary>
    private float Cosh(float x)
    {
        float ex = Mathf.Exp(x), enx = Mathf.Exp(-x);
        return 0.5f * (ex + enx);
    }

    #endregion
} 