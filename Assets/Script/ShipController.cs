using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class ShipController : MonoBehaviour
{
    [HideInInspector] public float shipMass;
    [HideInInspector] public Vector3 initialVelocity;

    private Rigidbody rb;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        if (rb == null) Debug.LogError("[ShipController] Rigidbody 컴포넌트를 찾을 수 없습니다!");
    }

    public void Initialize(float mass, Vector3 velocity)
    {
        shipMass = mass;
        initialVelocity = velocity;
        rb.mass = shipMass;
        rb.velocity = initialVelocity;
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        rb.useGravity = false;
        rb.isKinematic = false;
    }
}