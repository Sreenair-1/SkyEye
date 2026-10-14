// SkyEye - Falling relief kit.
using UnityEngine;

namespace SkyEye
{
    /// <summary>Relief kit falling under gravity until it lands.</summary>
    public class SupplyDrop : MonoBehaviour
    {
        public float groundY;
        float vy;

        void Update()
        {
            if (transform.position.y <= groundY) return;
            vy -= 9.81f * Time.deltaTime;
            var p = transform.position;
            p.y = Mathf.Max(groundY, p.y + vy * Time.deltaTime);
            transform.position = p;
        }
    }
}
