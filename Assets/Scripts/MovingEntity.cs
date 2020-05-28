using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MovingEntity : BaseGameEntity
{
    /// <summary>
    /// 最大速度
    /// </summary>
    public float m_MaxSpeed;
    /// <summary>
    /// 最大受力
    /// </summary>
    public float m_MaxForce;
    /// <summary>
    /// 最大角速度
    /// </summary>
    public float m_MaxTurnRate;
    /// <summary>
    /// 恐慌距离
    /// </summary>
    public float PanicDistance;
    /// <summary>
    /// 视角
    /// </summary>
    public float sightAngle;
    /// <summary>
    /// 视野半径
    /// </summary>
    public float sightRadius;

    public Vector2 velocity
    {
        get
        {
            return GetComponent<Rigidbody2D>().velocity;
        }
    }
    public Vector2 head
    {
        get
        {
            return transform.up;
        }
    }
    public Vector2 right
    {
        get
        {
            return transform.right;
        }
    }
    public float rotation_z
    {
        get
        {
            return transform.rotation.z;
        }
    }


    
}
