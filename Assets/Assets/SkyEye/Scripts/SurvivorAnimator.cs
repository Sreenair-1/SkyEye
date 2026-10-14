// SkyEye - Survivor waving for help (simple time-based animation).
using UnityEngine;

namespace SkyEye
{
    public class SurvivorAnimator : MonoBehaviour
    {
        public Transform leftArm, rightArm;

        void Update()
        {
            float a = Mathf.Sin(Time.time * 6f) * 35f;
            if (leftArm) leftArm.localRotation = Quaternion.Euler(0f, 0f, 150f + a);
            if (rightArm) rightArm.localRotation = Quaternion.Euler(0f, 0f, -150f - a);
        }
    }
}
