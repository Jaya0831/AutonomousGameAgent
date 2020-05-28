using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Vehicle : MovingEntity
{
    /// <summary>
    /// 指向操作类
    /// </summary>
    public SteeringBehavior m_Steering;
    
    public GameObject wanderTarget;

    public GameObject obj;

    private void Awake()
    {
        initID();
        VehicleManager.RegisterVehicle(this);
        m_Steering = new SteeringBehavior();
        m_Steering.m_vehicle = this;
        m_MaxSpeed =7;
        m_MaxForce = 7;
        m_MaxTurnRate=1000;
        PanicDistance=10;
        sightAngle = 2.0f;//大概120度
        sightRadius = 10f;
        wanderTarget.transform.SetPositionAndRotation(transform.position, Quaternion.identity);
    }

    //平滑
    private List<float> angles;
    private float smoothAngles;
    private float smoothTimes = 5;


    private void Start()
    {
        smoothAngles = Mathf.Atan2(head.y, head.x);
        for (int i = 0; i < smoothTimes; i++)
        {
            angles.Add(smoothAngles);
        }
    }

    private void Update()
    {
        // 以速度方向更新朝向
        if (velocity.magnitude > 0.1f)
        {
            transform.up = Smooth();
        }

        Vector2 SteeringForce = m_Steering.Calculate();
        if (SteeringForce.magnitude > m_MaxForce) 
        {
            SteeringForce = SteeringForce.normalized * m_MaxForce;
        }
        GetComponent<Rigidbody2D>().AddForce(SteeringForce);
        if (GetComponent<Rigidbody2D>().velocity.magnitude > m_MaxSpeed)
        {
            GetComponent<Rigidbody2D>().velocity = GetComponent<Rigidbody2D>().velocity.normalized * m_MaxSpeed;
        }
    }
    /// <summary>
    /// 平滑处理
    /// </summary>
    /// <returns></returns>
    private Vector2 Smooth()
    {
        float deleteAngle = angles[0];
        angles.RemoveAt(0);
        if (velocity.magnitude > 0.01f)
        {
            angles.Add(Mathf.Atan2(velocity.normalized.y, velocity.normalized.x));
            smoothAngles = (smoothAngles * smoothTimes - deleteAngle + Mathf.Atan2(velocity.normalized.y, velocity.normalized.x)) / (float)smoothTimes;
        }
        else
        {
            angles.Add(Mathf.Atan2(head.y, head.x));
            smoothAngles = (smoothAngles * smoothTimes - deleteAngle + Mathf.Atan2(head.y, head.x)) / (float)smoothTimes;
        }

        return new Vector2(Mathf.Cos(smoothAngles), Mathf.Sin(smoothAngles));
    }
}
