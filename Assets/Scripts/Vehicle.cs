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


    private void Update()
    {
        // 以速度方向更新朝向
        if (velocity.magnitude > 0.1f)
        {
            //transform.up = Smooth();
            transform.up = velocity.normalized;
        }

        Vector2 SteeringForce = m_Steering.Calculate();
        GetComponent<Rigidbody2D>().AddForce(SteeringForce);
        if (GetComponent<Rigidbody2D>().velocity.magnitude > m_MaxSpeed)
        {
            GetComponent<Rigidbody2D>().velocity = GetComponent<Rigidbody2D>().velocity.normalized * m_MaxSpeed;
        }
    }
}
