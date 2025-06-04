using UnityEngine;

[RequireComponent(typeof(Camera))]
public class CameraManager : MonoBehaviour
{
    [Header("Simulation Manager")]
    public SimulationManager sim;

    [Header("Targets")]
    public Transform primaryTarget;
    public Transform secondaryTarget;

    [Header("Camera Settings")]
    [Range(1f, 179f)]
    public float referenceFOV = 60f;
    public float minFOV = 1f;
    public float maxFOV = 179f;

    [Header("Keep Centered on Ship Height")]
    public bool keepCentered = false;

    [Header("Offset (Unity 단위, 1유닛=10km)")]
    public Vector3 offset = new Vector3(0f, 0f, -20f);

    private Camera cam;
    private Quaternion fixedRotation;

    void Start()
    {
        cam = GetComponent<Camera>();
        if (cam == null) Debug.LogError("[CameraManager] Camera 컴포넌트를 찾을 수 없습니다!");
        cam.orthographic = false;
        cam.nearClipPlane = 0.0001f;
        cam.farClipPlane = 1e9f;
        cam.eventMask = 0;
        fixedRotation = transform.rotation;
    }

    void LateUpdate()
    {
        if (sim == null || primaryTarget == null) return;

        cam.fieldOfView = referenceFOV;
        Vector3 basePos = primaryTarget.position;
        if (keepCentered && secondaryTarget != null)
        {
            basePos = new Vector3(basePos.x, secondaryTarget.position.y, basePos.z);
        }
        transform.position = basePos + offset;
        transform.rotation = fixedRotation;
    }
}