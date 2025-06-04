using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

public class UIManager : MonoBehaviour
{
    [SerializeField] private UIDocument uiDocument;
    [SerializeField] private UIDocument stopPanelDocument;
    [SerializeField] private UIDocument logWindowDocument;

    private ScrollView inputPanel;
    private Button startBtn, stopBtn, resumeBtn;
    private ScrollView logWindow;
    private Label logLabel, resultLabel;
    private SimulationManager sim;
    private Camera cam;

    private int previousCamMode;
    private Vector3 initialCamPos;
    private Quaternion initialCamRot;
    private Quaternion stopCamRot;
    private Queue<string> logLines = new Queue<string>(100);

    void Awake()
    {
        sim = SimulationManager.Instance ?? FindObjectOfType<SimulationManager>();
        if (sim == null) Debug.LogError("[UIManager] SimulationManager를 찾을 수 없습니다!");
        cam = Camera.main ?? GetComponent<Camera>();
        if (cam == null) Debug.LogError("[UIManager] Camera를 찾을 수 없습니다!");
        sim.OnLogUpdated += UpdateLog;
        sim.OnSpeedUpdated += UpdateSpeed;
        Debug.Log("[UIManager] Awake: 초기화 완료, 이벤트 연결");
    }

    void OnEnable()
    {
        var root = uiDocument.rootVisualElement;
        inputPanel = root.Q<ScrollView>(className: "input-panel");
        startBtn = root.Q<Button>("startBtn");
        resultLabel = root.Q<Label>("result");
        startBtn.clicked += OnStart;

        var stopRoot = stopPanelDocument.rootVisualElement;
        stopBtn = stopRoot.Q<Button>("stopBtn");
        resumeBtn = stopRoot.Q<Button>("resumeBtn");
        stopBtn.clicked += OnStop;
        resumeBtn.clicked += OnResume;
        stopRoot.style.display = DisplayStyle.None;
        stopRoot.style.position = Position.Absolute;
        stopRoot.style.left = 0;
        stopRoot.style.top = 0;
        stopRoot.style.width = 150;
        stopRoot.pickingMode = PickingMode.Position;

        var logRoot = logWindowDocument.rootVisualElement;
        logWindow = logRoot.Q<ScrollView>("logWindow");
        logLabel = logRoot.Q<Label>("logLabel");
        logWindow.style.display = DisplayStyle.None;
        Debug.Log("[UIManager] OnEnable: UI 요소 바인딩 완료");
    }

    private void OnStart()
    {
        initialCamPos = new Vector3(10000f, 0f, -1000f);
        initialCamRot = Quaternion.Euler(30f, 0f, 0f);
        cam.transform.position = initialCamPos;
        cam.transform.rotation = initialCamRot;

        ReadAndApplyInputs();
        sim.StartSimulation();

        inputPanel.style.display = DisplayStyle.None;
        startBtn.style.display = DisplayStyle.None;
        stopPanelDocument.rootVisualElement.style.display = DisplayStyle.Flex;
        logWindow.style.display = DisplayStyle.Flex;

        ConfigureCamera();

        logLines.Clear();
        logLabel.text = "";
        Debug.Log("[UIManager] OnStart: 시뮬레이션 시작, UI 전환");
    }

    private void OnStop()
    {
        sim.StopSimulation();
        stopCamRot = cam.transform.rotation;

        if (cam.TryGetComponent<AutoFramer>(out var af)) af.enabled = false;
        if (cam.TryGetComponent<CameraManager>(out var cm)) cm.enabled = false;

        var fc = cam.GetComponent<FreeCameraController>()
                 ?? cam.gameObject.AddComponent<FreeCameraController>();
        fc.moveSpeed = 50f;
        fc.rotateSpeed = 200f;
        fc.zoomSpeed = 5f;
        fc.enabled = true;
        Debug.Log("[UIManager] OnStop: 시뮬레이션 정지, 자유 카메라 활성화");
    }

    private void OnResume()
    {
        sim.camMode = previousCamMode;
        sim.ResumeSimulation();
        ConfigureCamera();
        cam.transform.rotation = stopCamRot;
        Debug.Log("[UIManager] OnResume: 시뮬레이션 재개, 카메라 복원");
    }

    private void ReadAndApplyInputs()
    {
        var r = uiDocument.rootVisualElement;
        sim.planetMass = Mathf.Max(r.Q<FloatField>("planetMass").value, 1e10f);
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
        sim.shipMass = Mathf.Max(r.Q<FloatField>("shipMass").value, 1e3f);
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
        sim.gravConst = Mathf.Max(r.Q<FloatField>("gravConst").value, 1e-12f);
        sim.timeScale = r.Q<FloatField>("timeScale").value;
        sim.trajWidth = r.Q<FloatField>("relWidth").value;
        sim.simulationMethod = r.Q<DropdownField>("simMethod").index;
        sim.camMode = r.Q<DropdownField>("camMode").index;
        sim.isPointMass = r.Q<DropdownField>("planetSizeMode").index == 0;
        sim.planetRadius = Mathf.Max(r.Q<FloatField>("planetRadius").value, 0f);

        // ★ Unit Scale 읽어오기 (MainUI.uxml에서 입력한 단위값)
        sim.unitScale = Mathf.Max(r.Q<FloatField>("unitScale").value, 1f);

        previousCamMode = sim.camMode;
        Debug.Log($"[UIManager] ReadAndApplyInputs: planetMass={sim.planetMass}, shipPos={sim.shipPos}, shipVel={sim.shipVel}, camMode={sim.camMode}, unitScale={sim.unitScale}");
    }

    private void ConfigureCamera()
    {
        if (cam.TryGetComponent<FreeCameraController>(out var fc)) fc.enabled = false;
        if (cam.TryGetComponent<AutoFramer>(out var af)) af.enabled = false;
        if (cam.TryGetComponent<CameraManager>(out var cm)) cm.enabled = false;

        if (sim.camMode == 0)
        {
            var af2 = cam.GetComponent<AutoFramer>() ?? cam.gameObject.AddComponent<AutoFramer>();
            af2.sim = sim;
            af2.target = sim.ShipTransform;
            af2.enabled = true;
            Debug.Log("[UIManager] ConfigureCamera: AutoFramer 활성화");
        }
        else
        {
            var cm2 = cam.GetComponent<CameraManager>() ?? cam.gameObject.AddComponent<CameraManager>();
            cm2.sim = sim;
            cm2.primaryTarget = sim.PlanetTransform;
            cm2.secondaryTarget = sim.camMode == 2 ? sim.ShipTransform : null;
            cm2.keepCentered = sim.camMode == 1;
            cm2.offset = new Vector3(0f, 0f, -20f);
            cm2.enabled = true;
            Debug.Log("[UIManager] ConfigureCamera: CameraManager 활성화");
        }
    }

    private void UpdateLog(string log)
    {
        logLines.Enqueue(log);
        if (logLines.Count > 100) logLines.Dequeue();
        logLabel.text = string.Join("\n", logLines);
        logWindow.schedule.Execute(() => logWindow.verticalScroller.value = logWindow.verticalScroller.highValue);
        Debug.Log($"[UIManager] UpdateLog: {log}");
    }

    private void UpdateSpeed(float speed)
    {
        resultLabel.text = $"Δv = {speed:0.000} km/s";
        logLines.Enqueue($"Speed: {speed:0.000} km/s");
        if (logLines.Count > 100) logLines.Dequeue();
        logLabel.text = string.Join("\n", logLines);
        logWindow.schedule.Execute(() => logWindow.verticalScroller.value = logWindow.verticalScroller.highValue);
        Debug.Log($"[UIManager] UpdateSpeed: Δv = {speed:0.000} km/s");
    }

    void OnDisable()
    {
        sim.OnLogUpdated -= UpdateLog;
        sim.OnSpeedUpdated -= UpdateSpeed;
        Debug.Log("[UIManager] OnDisable: 이벤트 해제");
    }
}
