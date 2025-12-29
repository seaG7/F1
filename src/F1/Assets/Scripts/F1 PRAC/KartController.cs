using System.Collections;
using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(Rigidbody))]
public class KartController : MonoBehaviour
{
    public static KartController Instance;

    [SerializeField] private KartConfiguration _config;

    [Header("Physics parameters")]
    [SerializeField] private float _gravity = 9.81f;

    [Header("Wheels")]
    [SerializeField] private Transform _wheelsParent;
    private Transform _frontLeftWheel;
    private Transform _frontRightWheel;
    private Transform _rearLeftWheel;
    private Transform _rearRightWheel;

    [Header("Weight distribution")]
    [Range(0f, 1f)]
    [SerializeField] private float _frontAxleShare = 0.5f;

    [Header("Steering")]
    [SerializeField] private float _maxSteerAngle = 30f;

    [Header("Input Actions")]
    [SerializeField] private InputActionReference _moveActionRef;
    [SerializeField] private InputActionReference _handbrakeActionRef;
    [SerializeField] private InputActionAsset inputActions;

    [Header("Engine")]
    [SerializeField] private KartEngine _engine;
    [SerializeField] private float _gearRatio = 8f;
    [SerializeField] private float _drivetrainEfficiency = 0.9f;
    [SerializeField] private float _wheelRadius = 0.3f;

    [Header("Rolling resistance")]
    [SerializeField] private float _rollingResistance = 0.5f;
    [SerializeField] private float _handbrakeRollingMultiplier = 3f;

    [Header("Tyre friction")]
    [SerializeField] private float _frictionCoefficient = 1.0f;
    [SerializeField] private float _frontLateralStiffness = 80f;
    [SerializeField] private float _rearLateralStiffness = 100f;

    [Header("Handbrake tyre settings")]
    [SerializeField] private float _rearLateralStiffnessWithHandbrake = 0f;

    [Header("Ground Check")]
    [SerializeField] private Vector3 groundRayOffset = new Vector3(0, 0.5f, 0);
    public LayerMask groundLayer;
    public float groundRayLength = 1.5f;

    [Header("Aerodynamics")]
    [SerializeField] private float _dragCoefficient = 0.5f;
    [SerializeField] private float _frontalArea = 2.5f;
    [SerializeField] private float _downforceCoefficient = 0.8f;
    [SerializeField] private float _airDensity = 1.225f;

    private Rigidbody _rb;
    private float _frontLeftNormalForce;
    private float _frontRightNormalForce;
    private float _rearLeftNormalForce;
    private float _rearRightNormalForce;

    [HideInInspector] public float[] wheelDistances = new float[4];
    private float[] _suspensionForces = new float[4];
    private float[] _suspensionCompressions = new float[4];
    
    [HideInInspector] public float[] suspSpringForces = new float[4];
    [HideInInspector] public float[] suspDamperForces = new float[4];
    [HideInInspector] public float[] suspTotalForces = new float[4];

    private Quaternion _frontLeftInitialLocalRot;
    private Quaternion _frontRightInitialLocalRot;

    private float _throttleInput;
    private float _steerInput;
    private bool _handbrakePressed;

    private float _currentDragForce;
    private float _currentDownforce;
    private float _currentGroundEffectForce;

    private float _wingArea;
    private float _wingAngle;
    private float _liftSlope;
    private float _groundFactor;
    private float _groundMaxDist;

    public bool IsGrounded { get; private set; }

    public float SpeedMs
    {
        get
        {
            if (_rb) return _rb.linearVelocity.magnitude;
            else return 0f;
        }
    }

    public float FrontAxleFy { get; private set; }
    public float RearAxleFx { get; private set; }
    public float FrontLeftVLat { get; private set; }
    public float FrontRightVLat { get; private set; }
    public float RearLeftVLat { get; private set; }
    public float RearRightVLat { get; private set; }

    private void Awake()
    {
        if (Instance == null)
            Instance = this;

        _rb = GetComponent<Rigidbody>();
        if (_config) ApplyConfig();
    }

    private void ApplyConfig()
    {
        _dragCoefficient = _config.dragCoefficient;
        _frontalArea = _config.frontalArea;
        _airDensity = _config.airDensity;
        _downforceCoefficient = _config.downforceCoefficient;
        _wingArea = _config.wingArea;
        _wingAngle = _config.wingAngleDeg;
        _liftSlope = _config.liftCoefficientSlope;
        _groundFactor = _config.groundEffectFactor;
        _groundMaxDist = _config.groundEffectMaxDist;
    }

    private void OnEnable()
    {
        InputActionMap actionMap = inputActions.FindActionMap("Kart");
        actionMap.Enable();

        if (_moveActionRef && _moveActionRef.action != null)
            _moveActionRef.action.Enable();

        if (_handbrakeActionRef && _handbrakeActionRef.action != null)
            _handbrakeActionRef.action.Enable();
    }

    private void Start()
    {
        StartCoroutine(EnableInputFix());

        if (_frontLeftWheel)
            _frontLeftInitialLocalRot = _frontLeftWheel.localRotation;

        if (_frontRightWheel)
            _frontRightInitialLocalRot = _frontRightWheel.localRotation;

        ComputeStaticWheelLoads();
        AttachWheels();
    }

    private void FixedUpdate()
    {
        if (!_rb) return;

        CheckGround();
        ApplyAerodynamics();

        if (IsGrounded)
        {
            FrontAxleFy = 0f;
            RearAxleFx = 0f;

            float speedAlongForward = Vector3.Dot(_rb.linearVelocity, transform.forward);
            float throttleAbs = Mathf.Abs(_throttleInput);
            float engineTorque = 0f;
            if (_engine)
            {
                engineTorque = _engine.Simulate(throttleAbs, speedAlongForward, Time.fixedDeltaTime);
            }

            float driveSign = Mathf.Sign(_throttleInput);
            float totalWheelTorque = engineTorque * _gearRatio * _drivetrainEfficiency;
            float wheelTorquePerRearWheel = totalWheelTorque * 0.5f;
            float driveForcePerRearWheel = _wheelRadius > 0.0001f ? driveSign * (wheelTorquePerRearWheel / _wheelRadius) : 0f;

            ApplyWheelForces(_frontLeftWheel, _frontLeftNormalForce, false, false, 0f, out float flFy, out float flVLat);
            ApplyWheelForces(_frontRightWheel, _frontRightNormalForce, false, false, 0f, out float frFy, out float frVLat);
            ApplyWheelForces(_rearLeftWheel, _rearLeftNormalForce, true, true, driveForcePerRearWheel, out float rlFy, out float rlVLat);
            ApplyWheelForces(_rearRightWheel, _rearRightNormalForce, true, true, driveForcePerRearWheel, out float rrFy, out float rrVLat);

            FrontAxleFy = flFy + frFy;
            FrontLeftVLat = flVLat;
            FrontRightVLat = frVLat;
            RearLeftVLat = rlVLat;
            RearRightVLat = rrVLat;
        }
        else
        {
            ApplyAirStabilization();
        }
    }

    private void Update()
    {
        ReadInput();
        RotateFrontWheels();
    }

    private void ApplyAerodynamics()
    {
        float v = SpeedMs;
        float vSq = v * v;

        _currentDragForce = 0.5f * _airDensity * _dragCoefficient * _frontalArea * vSq;
        if (v > 0.1f)
            _rb.AddForce(-_rb.linearVelocity.normalized * _currentDragForce);

        float cl = _liftSlope * (_wingAngle * Mathf.Deg2Rad);
        float wingForce = 0.5f * _airDensity * cl * _wingArea * vSq;
        _currentDownforce = wingForce + (0.5f * _airDensity * vSq * _downforceCoefficient * _frontalArea * 0.1f);
        _rb.AddForce(-transform.up * _currentDownforce);

        _currentGroundEffectForce = 0f;
        RaycastHit hit;
        if (Physics.Raycast(transform.position, -transform.up, out hit, _groundMaxDist, groundLayer))
        {
            float h = Mathf.Max(hit.distance, 0.05f);
            _currentGroundEffectForce = _groundFactor / h;
            _rb.AddForce(-transform.up * _currentGroundEffectForce);
        }
    }

    #region GUI Logic

    private GUIStyle _headerStyle, _valueStyle, _labelStyle;
    private Texture2D _whiteTexture;
    private bool _stylesInit;

    private void InitStyles()
    {
        if (_whiteTexture == null)
        {
            _whiteTexture = new Texture2D(1, 1);
            _whiteTexture.SetPixel(0, 0, Color.white);
            _whiteTexture.Apply();
        }

        if (_headerStyle == null)
        {
            _headerStyle = new GUIStyle(GUI.skin.label);
            _headerStyle.fontSize = 18; 
            _headerStyle.fontStyle = FontStyle.Bold;
            _headerStyle.alignment = TextAnchor.MiddleCenter;
            _headerStyle.normal.textColor = Color.white;
        }

        if (_valueStyle == null)
        {
            _valueStyle = new GUIStyle(GUI.skin.label);
            _valueStyle.fontSize = 40;
            _valueStyle.fontStyle = FontStyle.Bold;
            _valueStyle.alignment = TextAnchor.UpperRight;
            _valueStyle.normal.textColor = Color.cyan;
        }

        if (_labelStyle == null)
        {
            _labelStyle = new GUIStyle(GUI.skin.label);
            _labelStyle.fontSize = 12;
            _labelStyle.fontStyle = FontStyle.Bold;
            _labelStyle.normal.textColor = Color.white;
        }
        
        _stylesInit = true;
    }

    private void OnGUI()
    {
        if (_engine == null || !_rb) return;
        if (!_stylesInit) InitStyles();

        float hudWidth = 360;
        float hudHeight = 520;
        float margin = 20;
        float sectionGap = 15;

        Rect panelRect = new Rect(Screen.width - hudWidth - margin, Screen.height - hudHeight - margin, hudWidth, hudHeight);

        GUI.color = new Color(0, 0, 0, 0.7f);
        GUI.DrawTexture(panelRect, _whiteTexture);
        GUI.color = Color.white;

        GUILayout.BeginArea(panelRect);

        float currentY = 10;

        DrawShadowText(new Rect(0, currentY, hudWidth, 30), "F1 TELEMETRY", _headerStyle, Color.white);
        currentY += 40;

        float kmh = SpeedMs * 3.6f;
        DrawShadowText(new Rect(20, currentY, 180, 50), $"{Mathf.Abs(kmh):0}", _valueStyle, Color.cyan);
        GUI.Label(new Rect(210, currentY + 10, 100, 20), "KM/H", _labelStyle);
        GUI.Label(new Rect(210, currentY + 25, 100, 20), $"{SpeedMs:F1} m/s", _labelStyle);
        currentY += 60;

        float rpm = _engine.CurrentRpm;
        float maxRpm = _engine.MaxRpm;
        float rpmPercent = Mathf.Clamp01(rpm / maxRpm);

        GUI.Label(new Rect(10, currentY, 330, 20), $"Engine RPM: {rpm:0}", _labelStyle);
        currentY += 20;

        Rect rpmBgRect = new Rect(10, currentY, 340, 12);
        GUI.color = new Color(0.2f, 0.2f, 0.2f, 1f);
        GUI.DrawTexture(rpmBgRect, _whiteTexture);

        Rect rpmFillRect = new Rect(10, currentY, 340 * rpmPercent, 12);
        Color rpmColor = Color.Lerp(Color.green, Color.red, rpmPercent);
        if (rpm > maxRpm - 500) rpmColor = new Color(1f, 0.2f, 0f); 
        GUI.color = rpmColor;
        GUI.DrawTexture(rpmFillRect, _whiteTexture);
        GUI.color = Color.white;
        currentY += 12 + sectionGap;

        GUI.Label(new Rect(0, currentY, hudWidth, 20), "AERODYNAMICS", _headerStyle);
        currentY += 25;
        
        float col1 = 10;
        float col2 = 180;
        
        DrawParamCompact(new Rect(col1, currentY, 160, 20), "Drag Force:", $"{_currentDragForce:F0} N");
        DrawParamCompact(new Rect(col2, currentY, 160, 20), "Downforce:", $"{_currentDownforce:F0} N");
        currentY += 22;
        
        DrawParamCompact(new Rect(col1, currentY, 160, 20), "Ground Eff:", $"{_currentGroundEffectForce:F0} N");
        DrawParamCompact(new Rect(col2, currentY, 160, 20), "CoM Height:", $"{_rb.worldCenterOfMass.y:F3} m");
        currentY += 22 + sectionGap;

        GUI.Label(new Rect(0, currentY, hudWidth, 20), "SUSPENSION DATA", _headerStyle);
        currentY += 30;

        float[] xPos = { 10, 55, 120, 185, 250, 310 };
        GUI.color = Color.gray;
        GUI.Label(new Rect(xPos[0], currentY, 40, 20), "Whl", _labelStyle);
        GUI.Label(new Rect(xPos[1], currentY, 60, 20), "Spr", _labelStyle);
        GUI.Label(new Rect(xPos[2], currentY, 60, 20), "Dmp", _labelStyle);
        GUI.Label(new Rect(xPos[3], currentY, 60, 20), "Tot", _labelStyle);
        GUI.Label(new Rect(xPos[4], currentY, 50, 20), "Dst", _labelStyle);
        GUI.Label(new Rect(xPos[5], currentY, 40, 20), "Cmp%", _labelStyle);
        GUI.color = Color.white;
        currentY += 25;

        float rowStep = 24;
        DrawSuspensionRow(currentY, "FR", 0, xPos); currentY += rowStep;
        DrawSuspensionRow(currentY, "FL", 1, xPos); currentY += rowStep;
        DrawSuspensionRow(currentY, "RR", 2, xPos); currentY += rowStep;
        DrawSuspensionRow(currentY, "RL", 3, xPos); currentY += rowStep + sectionGap;

        float bottomY = currentY;

        DrawParamCompact(new Rect(10, bottomY, 150, 20), "Throttle:", $"{_throttleInput:F2}");
        DrawParamCompact(new Rect(10, bottomY + 25, 150, 20), "Steer:", $"{_steerInput:F2}");
        DrawParamCompact(new Rect(10, bottomY + 50, 150, 20), "Torque:", $"{_engine.CurrentTorque:F0} Nm");

        Rect gForceRect = new Rect(250, bottomY, 80, 80); 
        GUI.color = new Color(1, 1, 1, 0.1f);
        GUI.DrawTexture(gForceRect, _whiteTexture);
        
        Vector2 center = gForceRect.center;
        float mass = _rb.mass;
        float gScale = 35f; 
        float gX = RearAxleFx / (mass * 9.81f);
        float gLat = FrontAxleFy / (mass * 9.81f);

        Vector2 forcePos = center + new Vector2(gLat * gScale, -gX * gScale);
        forcePos.x = Mathf.Clamp(forcePos.x, gForceRect.x, gForceRect.xMax - 4);
        forcePos.y = Mathf.Clamp(forcePos.y, gForceRect.y, gForceRect.yMax - 4);

        GUI.color = Color.yellow;
        GUI.DrawTexture(new Rect(forcePos.x - 2, forcePos.y - 2, 4, 4), _whiteTexture);
        GUI.color = Color.white;
        
        GUI.Label(new Rect(250, bottomY + 82, 80, 20), "G-FORCE", _labelStyle);

        if (_handbrakePressed)
        {
            Rect hbRect = new Rect(10, hudHeight - 25, 340, 20); 
            GUI.color = Color.red;
            GUI.DrawTexture(hbRect, _whiteTexture);
            GUI.color = Color.white;
            GUI.Label(hbRect, "HANDBRAKE ACTIVE", _headerStyle);
        }

        GUILayout.EndArea();
    }


    void DrawShadowText(Rect rect, string text, GUIStyle style, Color color)
    {
        var backup = style.normal.textColor;
        style.normal.textColor = Color.black;
        GUI.Label(new Rect(rect.x + 2, rect.y + 2, rect.width, rect.height), text, style);
        style.normal.textColor = color;
        GUI.Label(rect, text, style);
        style.normal.textColor = backup;
    }

    void DrawParamCompact(Rect rect, string label, string value)
    {
        GUI.color = Color.gray;
        GUI.Label(new Rect(rect.x, rect.y, 90, rect.height), label, _labelStyle);
        GUI.color = Color.cyan;
        GUI.Label(new Rect(rect.x + 95, rect.y, 80, rect.height), value, _labelStyle);
        GUI.color = Color.white;
    }

    void DrawSuspensionRow(float y, string name, int idx, float[] xPos)
    {
        GUI.Label(new Rect(xPos[0], y, 40, 20), name, _labelStyle);
        
        GUI.color = Color.cyan;
        GUI.Label(new Rect(xPos[1], y, 60, 20), $"{suspSpringForces[idx]:0}", _labelStyle);
        GUI.Label(new Rect(xPos[2], y, 60, 20), $"{suspDamperForces[idx]:0}", _labelStyle);
        GUI.Label(new Rect(xPos[3], y, 60, 20), $"{suspTotalForces[idx]:0}", _labelStyle);
        
        float dist = wheelDistances[idx];
        GUI.color = dist >= 0.9f ? Color.red : Color.cyan;
        GUI.Label(new Rect(xPos[4], y, 50, 20), $"{dist:F2}", _labelStyle);

        GUI.color = Color.cyan;
        GUI.Label(new Rect(xPos[5], y, 40, 20), $"{_suspensionCompressions[idx]:0}", _labelStyle);
        
        GUI.color = Color.white;
    }

    #endregion

    private void AttachWheels()
    {
        _frontRightWheel = _wheelsParent.GetChild(0);
        _frontLeftWheel = _wheelsParent.GetChild(1);
        _rearRightWheel = _wheelsParent.GetChild(2);
        _rearLeftWheel = _wheelsParent.GetChild(3);
    }

    private void ComputeStaticWheelLoads()
    {
        if (!_rb)
            _rb = GetComponent<Rigidbody>();

        float mass = _rb.mass;
        float totalWeight = mass * _gravity;

        float frontWeight = totalWeight * _frontAxleShare;
        float rearWeight = totalWeight * (1f - _frontAxleShare);

        _frontLeftNormalForce = frontWeight * 0.5f;
        _frontRightNormalForce = frontWeight * 0.5f;

        _rearLeftNormalForce = rearWeight * 0.5f;
        _rearRightNormalForce = rearWeight * 0.5f;
    }

    private void ReadInput()
    {
        if (_moveActionRef && _moveActionRef.action != null)
        {
            Vector2 move = _moveActionRef.action.ReadValue<Vector2>();
            _steerInput = Mathf.Clamp(move.x, -1f, 1f);
            _throttleInput = Mathf.Clamp(move.y, -1f, 1f);
        }
        else
        {
            _steerInput = 0f;
            _throttleInput = 0f;
        }

        if (_handbrakeActionRef && _handbrakeActionRef.action != null)
        {
            float hb = _handbrakeActionRef.action.ReadValue<float>();
            _handbrakePressed = hb > 0.5f;
        }
        else
        {
            _handbrakePressed = false;
        }
    }

    private void RotateFrontWheels()
    {
        float steerAngle = _maxSteerAngle * _steerInput;
        Quaternion steerRotation = Quaternion.Euler(0f, steerAngle, 0f);

        if (_frontLeftWheel)
            _frontLeftWheel.localRotation = _frontLeftInitialLocalRot * steerRotation;

        if (_frontRightWheel)
            _frontRightWheel.localRotation = _frontRightInitialLocalRot * steerRotation;
    }

    private void ApplyWheelForces(
        Transform wheel,
        float normalForce,
        bool isDriven,
        bool isRear,
        float driveForceInput,
        out float fyOut,
        out float vLatOut)
    {
        fyOut = 0f;
        vLatOut = 0f;

        if (!wheel || !_rb)
            return;

        Vector3 wheelPos = wheel.position;
        Vector3 wheelForward = wheel.forward;
        Vector3 wheelRight = wheel.right;

        Vector3 v = _rb.GetPointVelocity(wheelPos);
        float vLong = Vector3.Dot(v, wheelForward);
        float vLat = Vector3.Dot(v, wheelRight);

        vLatOut = vLat;

        float Fx = 0f;
        float Fy = 0f;

        if (isDriven)
        {
            Fx += driveForceInput;
        }

        float rolling = _rollingResistance;
        if (isRear && _handbrakePressed)
        {
            rolling *= _handbrakeRollingMultiplier;
        }

        Fx += -rolling * vLong;

        float lateralStiffness;
        if (isRear) lateralStiffness = _rearLateralStiffness;
        else lateralStiffness = _frontLateralStiffness;
        if (isRear && _handbrakePressed)
        {
            lateralStiffness = _rearLateralStiffnessWithHandbrake;
        }

        Fy += -lateralStiffness * vLat;

        float frictionLimit = _frictionCoefficient * normalForce;
        float forceLength = Mathf.Sqrt(Fx * Fx + Fy * Fy);

        if (forceLength > frictionLimit && forceLength > 1e-6f)
        {
            float scale = frictionLimit / forceLength;
            Fx *= scale;
            Fy *= scale;
        }

        fyOut = Fy;

        Vector3 force = wheelForward * Fx + wheelRight * Fy;
        _rb.AddForceAtPosition(force, wheelPos, ForceMode.Force);

        if (isRear)
        {
            RearAxleFx += Vector3.Dot(force, transform.forward);
        }
    }

    private void ApplyAirStabilization()
    {
        Vector3 currentUp = transform.up;
        if (Vector3.Angle(currentUp, Vector3.up) > 10f)
        {
            Vector3 correctionTorque = Vector3.Cross(currentUp, Vector3.up) * 1500f;
            _rb.AddTorque(correctionTorque);
        }
    }

    private void CheckGround()
    {
        Vector3 rayStart = transform.position + groundRayOffset;

        int raysHit = 0;
        Vector3[] rayOffsets = new Vector3[]
        {
            new Vector3(1f, 0, 1f),
            new Vector3(-1f, 0, 1f),
            new Vector3(1f, 0, -1f),
            new Vector3(-1f, 0, -1f)
        };

        foreach (Vector3 offset in rayOffsets)
        {
            if (Physics.Raycast(rayStart + offset * 0.5f, -transform.up, groundRayLength, groundLayer))
            {
                raysHit++;
                Debug.DrawRay(rayStart + offset * 0.5f, -transform.up * groundRayLength, Color.green);
            }
            else
            {
                Debug.DrawRay(rayStart + offset * 0.5f, -transform.up * groundRayLength, Color.red);
            }
        }

        IsGrounded = raysHit >= 2;
    }

    public void SetSuspensionForce(int wheelIndex, float force)
    {
        if (wheelIndex >= 0 && wheelIndex < 4)
            _suspensionForces[wheelIndex] = force;
    }

    public void SetSuspensionCompression(int wheelIndex, float compression)
    {
        if (wheelIndex >= 0 && wheelIndex < 4)
            _suspensionCompressions[wheelIndex] = compression;
    }
    
    public void SetWheelDistance(int wheelIndex, float distance)
    {
        if (wheelIndex >= 0 && wheelIndex < 4)
            wheelDistances[wheelIndex] = distance;
    }

    private IEnumerator EnableInputFix()
    {
        yield return new WaitForSeconds(0.1f);

        gameObject.SetActive(false);
        gameObject.SetActive(true);

        yield return null;
    }
}