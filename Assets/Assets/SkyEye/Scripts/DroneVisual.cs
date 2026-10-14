// SkyEye - Drone model animation: spinning rotors and blinking navigation lights.
using UnityEngine;

namespace SkyEye
{
    public class DroneVisual : MonoBehaviour
    {
        public Transform[] rotors;
        public Renderer[] blinkers;
        public Material blinkOn, blinkOff;
        public float maxRpm = 1800f;

        float spin, angle;

        public void SetSpin(float amount) { spin = Mathf.Clamp01(amount); }

        void Update()
        {
            angle += maxRpm / 60f * 360f * spin * Time.deltaTime;
            if (rotors != null)
                for (int i = 0; i < rotors.Length; i++)
                    if (rotors[i] != null)
                        rotors[i].localRotation = Quaternion.Euler(0f, (i % 2 == 0 ? angle : -angle), 0f);
            bool on = Mathf.FloorToInt(Time.time * 2f) % 2 == 0;
            if (blinkers != null)
                foreach (var b in blinkers) if (b) b.sharedMaterial = on ? blinkOn : blinkOff;
        }
    }
}
