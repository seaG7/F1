using UnityEngine;

public class KartEngine : MonoBehaviour
{
    [SerializeField] private KartConfiguration _config;

    [Header("RPM settings")]
    [SerializeField] private float _idleRpm = 1000f;
    [SerializeField] private float _maxRpm = 8000f;
    [SerializeField] private float _revLimiterRpm = 7500f;

    [Header("Torque curve")]
    [SerializeField] private AnimationCurve _torqueCurve;
    [SerializeField] private float _fallbackTorque = 400f;
    [SerializeField] private float _flywheelInertia = 0.2f;
    [SerializeField] private float _throttleResponse = 5f;

    [Header("Losses & load")]
    [SerializeField] private float _engineFrictionCoeff = 0.02f;
    [SerializeField] private float _loadTorqueCoeff = 5f;

    public float CurrentRpm { get; private set; }
    public float CurrentTorque { get; private set; }
    public float SmoothedThrottle { get; private set; }
    public float RevLimiterFactor { get; private set; } = 1f;
    public float MaxRpm => _config ? _config.maxRpm : _maxRpm;

    private float _invInertiaFactor;

    private void Awake()
    {
        if (_config) ApplyConfig();
        
        CurrentRpm = _idleRpm;
        _invInertiaFactor = 60f / (2f * Mathf.PI * Mathf.Max(_flywheelInertia, 0.0001f));

        if (_torqueCurve == null || _torqueCurve.length == 0)
             _torqueCurve = new AnimationCurve(new Keyframe(0, 1000), new Keyframe(1, 8000));
    }

    private void ApplyConfig()
    {
        _idleRpm = _config.idleRpm;
        _maxRpm = _config.maxRpm;
        _revLimiterRpm = _config.revLimiterRpm;
        _fallbackTorque = _config.fallbackTorque;
        if (_config.torqueCurve != null && _config.torqueCurve.length > 0)
            _torqueCurve = _config.torqueCurve;
        _flywheelInertia = _config.flywheelInertia;
        _throttleResponse = _config.throttleResponse;
        _engineFrictionCoeff = _config.engineFrictionCoeff;
        _loadTorqueCoeff = _config.loadTorqueCoeff;
    }

    public float Simulate(float throttleInput, float forwardSpeed, float deltaTime)
    {
        float targetThrottle = Mathf.Clamp01(throttleInput);
        SmoothedThrottle = Mathf.MoveTowards(SmoothedThrottle, targetThrottle, _throttleResponse * deltaTime);

        UpdateRevLimiterFactor();

        float maxTorqueAtRpm = EvaluateTorqueCurve(CurrentRpm);
        float effectiveThrottle = SmoothedThrottle * RevLimiterFactor;
        float driveTorque = maxTorqueAtRpm * effectiveThrottle;

        float frictionTorque = _engineFrictionCoeff * CurrentRpm;
        float loadTorque = _loadTorqueCoeff * Mathf.Abs(forwardSpeed);

        float netTorque = driveTorque - frictionTorque - loadTorque;

        float rpmDot = netTorque * _invInertiaFactor;
        CurrentRpm += rpmDot * deltaTime;

        if (CurrentRpm < _idleRpm) CurrentRpm = _idleRpm;
        if (CurrentRpm > _maxRpm) CurrentRpm = _maxRpm;

        CurrentTorque = driveTorque;
        return CurrentTorque;
    }

    private float EvaluateTorqueCurve(float rpm)
    {
        float curveTorque = 0f;
        if (_torqueCurve != null && _torqueCurve.length > 0)
        {
            float t = Mathf.Clamp01(rpm / _maxRpm);
            curveTorque = _torqueCurve.Evaluate(t);
        }
        else
        {
            curveTorque = _fallbackTorque;
        }
        return curveTorque;
    }

    private void UpdateRevLimiterFactor()
    {
        if (CurrentRpm <= _revLimiterRpm)
        {
            RevLimiterFactor = 1f;
            return;
        }

        if (CurrentRpm >= _maxRpm)
        {
            RevLimiterFactor = 0f;
            return;
        }

        float t = (CurrentRpm - _revLimiterRpm) / (_maxRpm - _revLimiterRpm);
        RevLimiterFactor = 1f - t;
    }
}