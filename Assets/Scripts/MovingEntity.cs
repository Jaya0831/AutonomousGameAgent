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

    //平滑
    public List<Vector2> heads;
    public Vector2 smoothHeading;
    public int smoothtimes = 30;

    private void Start()
    {
        smoothHeading = head;
        for (int i = 0; i < smoothtimes; i++)
        {
            heads.Add(head);
        }
    }

    /// <summary>
    /// 平滑处理
    /// </summary>
    /// <returns></returns>
    private Vector2 Smooth()
    {
        Vector2 deleteHead = heads[0];
        heads.RemoveAt(0);
        if (velocity.magnitude > 0.01f)
        {
            heads.Add(velocity.normalized);
            smoothHeading = (smoothHeading * smoothtimes - deleteHead + velocity.normalized) / smoothtimes;
        }
        else
        {
            heads.Add(head);
            smoothHeading = (smoothHeading * smoothtimes - deleteHead + head) / smoothtimes;
        }

        return smoothHeading;
    }
}
