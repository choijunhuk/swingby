using UnityEngine;

[RequireComponent(typeof(LineRenderer))]
public class CelestialBody : MonoBehaviour
{
    [HideInInspector] public float trailWidth = 0.1f;
    private LineRenderer lr;

    void Awake()
    {
        lr = GetComponent<LineRenderer>();
        ApplyWidth();
    }

    public void SetTrailWidth(float width)
    {
        trailWidth = width;
        if (lr != null) lr.startWidth = lr.endWidth = trailWidth;
    }

    private void ApplyWidth()
    {
        if (lr != null) lr.startWidth = lr.endWidth = trailWidth;
    }
}