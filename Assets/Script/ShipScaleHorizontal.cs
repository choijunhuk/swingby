using UnityEngine;

public class ShipScaleHorizontal : MonoBehaviour
{
    [Header("Relative Width")]
    public float relativeWidth = 1f;

    void Start()
    {
        UpdateScale();
    }

    public void SetRelativeWidth(float width)
    {
        relativeWidth = width;
        UpdateScale();
    }

    private void UpdateScale()
    {
        transform.localScale = Vector3.one; // 고정 스케일 유지
    }
}