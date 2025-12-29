using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class CarSuspension : MonoBehaviour
{
    [SerializeField] private KartConfiguration _config;

    [SerializeField] private GameObject wheelPrefab;
    [SerializeField] private Transform wheelsParent;
    [SerializeField] private Vector2 frontWheelDistance = new Vector2(2, 2);
    [SerializeField] private Vector2 rearWheelDistance = new Vector2(2, 2);

    private Rigidbody rb;
    private GameObject[] wheelPrefabs = new GameObject[4];
    private Vector3[] wheels = new Vector3[4];
    private Vector3[] suspensionForce = new Vector3[4];
    private Vector3[] tractionForce = new Vector3[4];
    private float[] lastCompression = new float[4];

    private const float maxTraction = 240f;
    private const float traction = 120f;
    private float rpm = 0;

    private float _restLength = 1f;
    private float _travel = 0.2f;
    private float _stiffness = 20000f;
    private float _damper = 3500f;
    private float _radius = 0.3f;

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        if (_config) ApplyConfig();

        for (int i = 0; i < 4; i++)
        {
            wheelPrefab.transform.GetChild(1).localRotation = Quaternion.Euler(0, 180, 0);
            wheelPrefabs[i] = Instantiate(wheelPrefab, wheels[i], Quaternion.identity);
            wheelPrefabs[i].transform.parent = wheelsParent;
        }
    }

    private void ApplyConfig()
    {
        _restLength = _config.restLength;
        _travel = _config.springTravel;
        _stiffness = _config.springStiffness;
        _damper = _config.damperStiffness;
        _radius = _config.wheelRadius;
    }

    private void Update()
    {
        wheels[0] = transform.right * frontWheelDistance.x + transform.forward * frontWheelDistance.y; 
        wheels[1] = transform.right * -frontWheelDistance.x + transform.forward * frontWheelDistance.y; 
        wheels[2] = transform.right * rearWheelDistance.x + transform.forward * -rearWheelDistance.y; 
        wheels[3] = transform.right * -rearWheelDistance.x + transform.forward * -rearWheelDistance.y; 

        float dt = Time.deltaTime;

        for (int i = 0; i < 4; i++)
        {
            if (i == 0 || i == 1) wheelPrefabs[i].transform.eulerAngles = new Vector3(transform.eulerAngles.x, transform.eulerAngles.y + Input.GetAxisRaw("Horizontal") * 45f, transform.eulerAngles.z);
            else wheelPrefabs[i].transform.rotation = transform.rotation;

            rpm += Input.GetAxisRaw("Vertical") * dt;
            rpm -= ((rpm * Mathf.PI) + -transform.InverseTransformDirection(rb.linearVelocity).z) * dt * 0.1f;
            
            float telemetryDist = 10f;
            if (Physics.Raycast(transform.position + wheels[i], -transform.up, out RaycastHit longHit, 10f))
            {
                telemetryDist = longHit.distance - _radius;
            }

            RaycastHit hit;

            float maxLen = _restLength + _travel;
            
            bool isHit = Physics.Raycast(transform.position + wheels[i], -transform.up, out hit, maxLen + _radius);

            float forceMag = 0f;
            float compPct = 0f;

            if (isHit)
            {
                wheelPrefabs[i].transform.position = hit.point + transform.up * _radius;
                wheelPrefabs[i].transform.GetChild(0).Rotate(0, -rpm, 0, Space.Self);

                float currentDist = Mathf.Clamp(hit.distance - _radius, 0, maxLen);

                float compression = _restLength - currentDist;
                
                float rate = (compression - lastCompression[i]) / dt;
                lastCompression[i] = compression;

                float fSpring = _stiffness * compression;
                float fDamper = _damper * rate;
                float fTotal = fSpring + fDamper;

                float upForce = Mathf.Max(0, fTotal);
                suspensionForce[i] = transform.up * upForce;
                
                forceMag = suspensionForce[i].magnitude;

                float totalTravelRange = _travel * 2f;
                float currentCompressionFromBottom = maxLen - currentDist;
                compPct = Mathf.Clamp01(currentCompressionFromBottom / totalTravelRange) * 100f;

                tractionForce[i] = Vector3.ClampMagnitude(traction
                    * (transform.right * -transform.InverseTransformDirection(rb.linearVelocity).x + transform.forward
                    * ((rpm * Mathf.PI) + -transform.InverseTransformDirection(rb.linearVelocity).z)), maxTraction);

                if (i == 0 || i == 1) tractionForce[i] = Quaternion.AngleAxis(45 * Input.GetAxisRaw("Horizontal"), Vector3.up) * tractionForce[i];

                rb.AddForceAtPosition((tractionForce[i] + suspensionForce[i]) * dt, hit.point);

                if (KartController.Instance != null)
                {
                    KartController.Instance.suspSpringForces[i] = fSpring;
                    KartController.Instance.suspDamperForces[i] = fDamper;
                    KartController.Instance.suspTotalForces[i] = fTotal;
                }
            }
            else
            {
                wheelPrefabs[i].transform.position = transform.position + wheels[i] - transform.up * maxLen;
                if (KartController.Instance != null)
                {
                    KartController.Instance.suspSpringForces[i] = 0;
                    KartController.Instance.suspDamperForces[i] = 0;
                    KartController.Instance.suspTotalForces[i] = 0;
                }
            }

            if (KartController.Instance != null)
            {
                KartController.Instance.SetSuspensionForce(i, forceMag);
                KartController.Instance.SetSuspensionCompression(i, compPct);
                KartController.Instance.SetWheelDistance(i, telemetryDist);
            }
        }
    }
}