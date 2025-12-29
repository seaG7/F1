using UnityEngine;

[CreateAssetMenu(fileName = "KartConfig", menuName = "Kart/Configuration")]
public class KartConfiguration : ScriptableObject
{
    [Header("Engine")]
    public float idleRpm = 1000f;
    public float maxRpm = 8000f;
    public float revLimiterRpm = 7500f;
    public float fallbackTorque = 500f;
    public AnimationCurve torqueCurve = new AnimationCurve(new Keyframe(0, 1000), new Keyframe(1, 8000));
    public float flywheelInertia = 0.2f;
    public float throttleResponse = 5f;
    public float engineFrictionCoeff = 0.02f;
    public float loadTorqueCoeff = 5f;

    [Header("Aerodynamics")]
    public float dragCoefficient = 0.5f;
    public float frontalArea = 2.5f;
    public float airDensity = 1.225f;
    public float downforceCoefficient = 0.8f;
    
    [Header("Wing Settings")]
    public float wingArea = 0.6f;
    public float wingAngleDeg = 10f;
    public float liftCoefficientSlope = 0.1f;

    [Header("Ground Effect")]
    public float groundEffectFactor = 50f;
    public float groundEffectMaxDist = 0.3f;

    [Header("Suspension")]
    public float restLength = 0.6f;
    public float springTravel = 0.3f;
    public float springStiffness = 20000f;
    public float damperStiffness = 3500f;
    public float wheelRadius = 0.3f;
}