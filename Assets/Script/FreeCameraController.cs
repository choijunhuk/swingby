using UnityEngine;

public class FreeCameraController : MonoBehaviour
{
    public float moveSpeed = 50f;
    public float rotateSpeed = 200f;
    public float zoomSpeed = 5f;

    private float yaw;
    private float pitch;

    void OnEnable()
    {
        yaw = transform.eulerAngles.y;
        pitch = transform.eulerAngles.x;
    }

    void Update()
    {
        float deltaTime = Time.deltaTime;
        float moveX = Input.GetAxis("Horizontal") * moveSpeed * deltaTime;
        float moveZ = Input.GetAxis("Vertical") * moveSpeed * deltaTime;
        float moveY = ((Input.GetKey(KeyCode.Q) ? 1f : 0f) - (Input.GetKey(KeyCode.E) ? 1f : 0f)) * moveSpeed * deltaTime;
        float zoom = Input.GetAxis("Mouse ScrollWheel") * zoomSpeed * 1000f * deltaTime;

        transform.Translate(moveX, moveY, moveZ, Space.Self);
        transform.Translate(0f, 0f, zoom, Space.Self);

        yaw += rotateSpeed * Input.GetAxis("Mouse X") * deltaTime;
        pitch -= rotateSpeed * Input.GetAxis("Mouse Y") * deltaTime;
        pitch = Mathf.Clamp(pitch, -90f, 90f);
        transform.eulerAngles = new Vector3(pitch, yaw, 0f);
    }
}