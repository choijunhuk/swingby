using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

public class UIManager : MonoBehaviour
{
    [SerializeField] private UIDocument uiDocument;
    [SerializeField] private UIDocument stopPanelDocument;
    [SerializeField] private UIDocument logWindowDocument;

    private ScrollView inputPanel;
    private Button    startBtn;
    private Button    stopBtn;
    private Button    resumeBtn;

    private ScrollView logWindow;
    private Label      logLabel;
    private Label      resultLabel;

    private SimulationManager sim;
    private Camera            cam;

    private Vector3    stopCamPos;
    private Quaternion stopCamRot;

    private Queue<string> logLines = new Queue<string>(100);
    private Toggle         applyKmFactorToggle;

    void Awake()
    {
        sim = SimulationManager.Instance ?? FindObjectOfType<SimulationManager>();
        if (sim == null)
            Debug.LogError("[UIManager] SimulationManager를 찾을 수 없습니다!");

        cam = Camera.main ?? GetComponent<Camera>();
        if (cam == null)
            Debug.LogError("[UIManager] Camera를 찾을 수 없습니다!");

        sim.OnLogUpdated   += UpdateLog;
        sim.OnSpeedUpdated += UpdateSpeed;

        Debug.Log("[UIManager] Awake: 초기화 완료, 이벤트 연결");
    }

    void OnEnable()
    {
        var root = uiDocument.rootVisualElement;

        inputPanel  = root.Q<ScrollView>(className: "input-panel");
        startBtn    = root.Q<Button>("startBtn");
        resultLabel = root.Q<Label>("result");

        startBtn.clicked += OnStart;

        applyKmFactorToggle = root.Q<Toggle>("applyKm");
        applyKmFactorToggle.RegisterValueChangedCallback(evt =>
        {
            bool applyKm = evt.newValue;
            Debug.Log("[UIManager] Apply km→m 토글 값: " + applyKm);
        });

        var stopRoot = stopPanelDocument.rootVisualElement;
        stopBtn   = stopRoot.Q<Button>("stopBtn");
        resumeBtn = stopRoot.Q<Button>("resumeBtn");
        stopBtn.clicked   += OnStop;
        resumeBtn.clicked += OnResume;

        // 정지/재시작 패널 초기 숨김
        stopRoot.style.display   = DisplayStyle.None;
        stopRoot.style.position  = Position.Absolute;
        stopRoot.style.left      = 0;
        stopRoot.style.top       = 0;
        stopRoot.style.width     = 150;
        stopRoot.pickingMode     = PickingMode.Position;

        var logRoot = logWindowDocument.rootVisualElement;
        logWindow  = logRoot.Q<ScrollView>("logWindow");
        logLabel   = logRoot.Q<Label>("logLabel");
        logWindow.style.display = DisplayStyle.None;

        Debug.Log("[UIManager] OnEnable: UI 요소 바인딩 완료");
    }

    private void OnStart()
    {
        // (0) 혹시 남아 있는 FreeCameraController, AutoFramer, CameraManager가 켜져 있으면 전부 비활성화
        if (cam.TryGetComponent<FreeCameraController>(out var fc))
            fc.enabled = false;
        if (cam.TryGetComponent<AutoFramer>(out var af))
            af.enabled = false;
        if (cam.TryGetComponent<CameraManager>(out var cm))
            cm.enabled = false;

        // (1) UI에서 입력을 읽어 시뮬레이션 설정
        ReadAndApplyInputs();

        // (2) 시뮬레이션 시작 → shipInstance 생성(Instantiate)
        sim.StartSimulation();

        // (3) camMode에 맞추어 FOV 제어 컴포넌트 활성화
        SwitchCamComponents();

        // (4) UI 전환
        inputPanel.style.display = DisplayStyle.None;
        startBtn.style.display   = DisplayStyle.None;
        stopPanelDocument.rootVisualElement.style.display = DisplayStyle.Flex;
        logWindow.style.display  = DisplayStyle.Flex;

        logLines.Clear();
        logLabel.text = "";

        Debug.Log("[UIManager] OnStart: 시뮬레이션 시작 → UI 전환");
    }

    private void OnStop()
    {
        sim.StopSimulation();

        stopCamPos = cam.transform.position;
        stopCamRot = cam.transform.rotation;

        // “정지” 시에만 FreeCameraController를 활성화
        var freeCam = cam.GetComponent<FreeCameraController>() 
                      ?? cam.gameObject.AddComponent<FreeCameraController>();
        freeCam.moveSpeed   = 50f;
        freeCam.rotateSpeed = 200f;
        freeCam.zoomSpeed   = 5f;
        freeCam.enabled     = true;

        // FOV 제어 컴포넌트는 꺼 두기
        if (cam.TryGetComponent<AutoFramer>(out var af2))
            af2.enabled = false;
        if (cam.TryGetComponent<CameraManager>(out var cm2))
            cm2.enabled = false;

        Debug.Log("[UIManager] OnStop: 시뮬레이션 정지 → FreeCameraController 활성화");
    }

    private void OnResume()
    {
        // FreeCameraController 비활성화
        if (cam.TryGetComponent<FreeCameraController>(out var freeCam))
            freeCam.enabled = false;

        sim.ResumeSimulation();

        // camMode에 맞추어 FOV 제어 컴포넌트 다시 활성화
        SwitchCamComponents();

        Debug.Log("[UIManager] OnResume: 시뮬레이션 재개 → FOV 컴포넌트 활성화");
    }

    private void ReadAndApplyInputs()
    {
        var r = uiDocument.rootVisualElement;

        sim.planetMass = r.Q<FloatField>("planetMass").value;
        sim.planetPos = new Vector3(
            r.Q<FloatField>("planetX").value,
            r.Q<FloatField>("planetY").value,
            r.Q<FloatField>("planetZ").value
        );
        sim.planetVel = new Vector3(
            r.Q<FloatField>("planetVelX").value,
            r.Q<FloatField>("planetVelY").value,
            r.Q<FloatField>("planetVelZ").value
        );

        sim.shipMass = r.Q<FloatField>("shipMass").value;
        sim.shipPos = new Vector3(
            r.Q<FloatField>("shipX").value,
            r.Q<FloatField>("shipY").value,
            r.Q<FloatField>("shipZ").value
        );
        sim.shipVel = new Vector3(
            r.Q<FloatField>("velX").value,
            r.Q<FloatField>("velY").value,
            r.Q<FloatField>("velZ").value
        );
        sim.shipRadius = Mathf.Max(r.Q<FloatField>("shipRadius").value, 0.01f);

        sim.gravConst = r.Q<FloatField>("gravConst").value;
        sim.timeScale = r.Q<FloatField>("timeScale").value;
        sim.trajWidth = r.Q<FloatField>("relWidth").value;
        sim.unitScale = r.Q<FloatField>("unitScale").value;

        sim.simulationMethod = r.Q<DropdownField>("simMethod").index;
        sim.camMode = r.Q<DropdownField>("camMode").index;
        sim.isPointMass = (r.Q<DropdownField>("planetSizeMode").index == 0);
        sim.planetRadius = r.Q<FloatField>("planetRadius").value;

        Debug.Log($"[UIManager] ReadAndApplyInputs: sim.camMode = {sim.camMode}");
    }

    private void SwitchCamComponents()
    {
        // camMode에 맞추어 FOV용 컴포넌트만 활성화
        if (cam.TryGetComponent<AutoFramer>(out var af))
            af.enabled = false;
        if (cam.TryGetComponent<CameraManager>(out var cm))
            cm.enabled = false;

        if (sim.camMode == 0)
        {
            // 우주선 기준 → AutoFramer 활성화
            if (!cam.TryGetComponent<AutoFramer>(out af))
                af = cam.gameObject.AddComponent<AutoFramer>();
            af.sim    = sim;
            af.target = sim.ShipTransform;
            af.enabled = true;
        }
        else if (sim.camMode == 1)
        {
            // 행성 기준 → CameraManager 활성화
            if (!cam.TryGetComponent<CameraManager>(out cm))
                cm = cam.gameObject.AddComponent<CameraManager>();
            cm.sim             = sim;
            cm.primaryTarget   = sim.ShipTransform;    // FOV만 사용
            cm.secondaryTarget = sim.PlanetTransform;   // FOV만 사용
            cm.enabled = true;
        }
    }

    // ---------------------------- 매 프레임 카메라를 대상 뒤(Z+10)에 고정하고 회전은 (0,180,0) ----------------------------
    void LateUpdate()
    {
        if (sim == null || !sim.IsRunning) return;

        if (sim.camMode == 0)
        {
            // 우주선 기준
            if (sim.ShipTransform != null)
            {
                Vector3 shipPos = sim.ShipTransform.position;
                cam.transform.position = shipPos + new Vector3(0f, 0f, -10f);
                // 회전을 (0,180,0)으로 고정
                cam.transform.rotation = Quaternion.Euler(0f, 0f, 0f);
            }
        }
        else if (sim.camMode == 1)
        {
            // 행성 기준
            if (sim.PlanetTransform != null)
            {
                Vector3 planetPos = sim.PlanetTransform.position;
                cam.transform.position = planetPos + new Vector3(0f, 0f, 10f);
                // 회전을 (0,180,0)으로 고정
                cam.transform.rotation = Quaternion.Euler(0f, 180f, 0f);
            }
        }
    }
    // -------------------------------------------------------------------------------------------------------

    private void UpdateLog(string log)
    {
        if (logLines.Count >= 100) logLines.Dequeue();
        logLines.Enqueue(log);
        logLabel.text = string.Join("\n", logLines.ToArray());
    }

    private void UpdateSpeed(float speed)
    {
        resultLabel.text = $"Δv = {speed:F3} km/s";
    }

    void OnDisable()
    {
        sim.OnLogUpdated   -= UpdateLog;
        sim.OnSpeedUpdated -= UpdateSpeed;
        Debug.Log("[UIManager] OnDisable: 이벤트 해제");
    }
}
