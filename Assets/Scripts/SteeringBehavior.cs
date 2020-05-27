using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SteeringBehavior : MonoBehaviour
{
    public Vehicle m_vehicle;
    public enum Deceleration { slow=3,normal=2,fast=1};
    /// <summary>
    /// 在以vehicle为圆心的圆上的位置
    /// </summary>
    public Vector2 m_WanderTarget;
    float m_WanderRadius = 1f;
    float m_WanderDistance = 2;
    float m_WanderJitter = 0.1f;
    float timeForAvoid = 1f;
    Vector2 m_SteeringForce;
    List<Vector2> m_Path;


    private void Start()
    {
        m_WanderTarget = new Vector2(0, m_WanderRadius);
    }
    public Vector2 Calculate()
    {
        //if (m_vehicle.m_ID == 0) 
        //{
        //    return OffsetPursuit(VehicleManager.GetVehicleFromID(1), new Vector2(-1, -1));
        //    //sight(VehicleManager.GetVehicleFromID(1));
        //}
        //return new Vector2(0, 0);
        float m_ValueOfWallAvoidance = 5;
        float m_ValueOfSeparation = 1;
        m_SteeringForce = new Vector2(0, 0);
        Vector2 force;
        force = WallAvoidance() * m_ValueOfWallAvoidance;
        if (!AccumulateForce(ref m_SteeringForce, force)) return m_SteeringForce;
        m_vehicle.TagNeighbors(m_vehicle, 5);
        force = LineAlignment(m_vehicle.neighborTag) * m_ValueOfSeparation;
        if (!AccumulateForce(ref m_SteeringForce, force)) return m_SteeringForce;
        return m_SteeringForce;
    }
    public bool AccumulateForce(ref Vector2 runningTot, Vector2 forceToAdd)
    {
        float magnitudeSoFar = runningTot.magnitude;
        float magnitudeRemaining = m_vehicle.m_MaxForce - magnitudeSoFar;
        if (magnitudeRemaining < 0.0) return false;
        float magnitudeToAdd = forceToAdd.magnitude;
        if (magnitudeToAdd < magnitudeRemaining)
        {
            runningTot += forceToAdd;
        }
        else
        {
            runningTot += (forceToAdd.normalized * magnitudeRemaining);
        }
        return true;
    }
    /// <summary>
    /// 将相对坐标转化成世界坐标
    /// </summary>
    /// <param name="local_vec"></param>
    /// <param name="position"></param>
    /// <param name="head"></param>
    /// <param name="left"></param>
    /// <returns></returns>
    public Vector2 FromLocalToWorld(Vector2 local_vec, Vector2 position, Vector2 head, Vector2 right)
    {
        Vector2 world_1 = new Vector2(local_vec.y * head.x, local_vec.y * head.y);
        Vector2 world_2 = new Vector2(local_vec.x * right.x, local_vec.x * right.y);
        return world_1 + world_2 + position;
    }
    /// <summary>
    /// 靠近
    /// </summary>
    /// <param name="target"></param>
    /// <returns></returns>
    public Vector2 Seek(Vector2 target)
    {
        Vector2 DesiredVelocity = (target - m_vehicle.position).normalized * m_vehicle.m_MaxSpeed;
        return (DesiredVelocity - m_vehicle.velocity).normalized * m_vehicle.m_MaxForce;
    }
    /// <summary>
    /// 离开
    /// </summary>
    /// <param name="target"></param>
    /// <returns></returns>
    public Vector2 Flee(Vector2 target)
    {
        if (Vector2.Distance(target, m_vehicle.position) > m_vehicle.PanicDistance)
        {
            return new Vector2(0, 0);
        }
        Debug.Log(Vector2.Distance(target, m_vehicle.position));
        Vector2 DesiredVelocity = (m_vehicle.position - target).normalized * m_vehicle.m_MaxSpeed;
        return (DesiredVelocity - m_vehicle.velocity).normalized * m_vehicle.m_MaxForce;
    } 
    /// <summary>
    /// 抵达
    /// </summary>
    /// <param name="target"></param>
    /// <param name="deceleration"></param>
    /// <returns></returns>
    public Vector2 Arrive(Vector2 target, Deceleration deceleration)
    {
        Vector2 ToTarget = target - m_vehicle.position;
        float dist = ToTarget.magnitude;
        if (dist > 0.1) 
        {
            float decelerationTweaker = 0.3f;
            float speed = Mathf.Min(dist / ((int)deceleration * decelerationTweaker), m_vehicle.m_MaxSpeed);//TODO：调参
            Vector2 desiredVelocity = ToTarget * speed / dist;
            return (desiredVelocity - m_vehicle.velocity).normalized * m_vehicle.m_MaxForce;
        }
        return new Vector2(0, 0);
    }
    /// <summary>
    /// 追逐
    /// </summary>
    /// <param name="evader"></param>
    /// <returns></returns>
    public Vector2 Pursuit(Vehicle evader)
    {
        Vector2 toEvader = (Vector2)evader.transform.position - m_vehicle.position;
        float RelativeHeading = m_vehicle.head.normalized.x * evader.head.normalized.x + m_vehicle.head.normalized.y * evader.head.normalized.y;
        if ((toEvader.normalized.x * m_vehicle.head.normalized.x + toEvader.normalized.y * m_vehicle.head.normalized.y > 0) && RelativeHeading < -0.95) //追逐者和逃避者相对
        {
            return Seek(evader.position);
        }
        //判断逃避者的位置
        float LookAheadTime = toEvader.magnitude / (m_vehicle.m_MaxSpeed + evader.velocity.magnitude);
        LookAheadTime += TurnaroundTime(m_vehicle, evader.position);
        return Seek(evader.position + LookAheadTime * evader.velocity);
    }
    /// <summary>
    /// 转向时间
    /// </summary>
    /// <param name="pAgent"></param>
    /// <param name="targetpos"></param>
    /// <returns></returns>
    public float TurnaroundTime(Vehicle pAgent, Vector2 targetpos)
    {
        Vector2 toTarget = (targetpos - pAgent.position).normalized;
        float dot = pAgent.head.normalized.x * toTarget.normalized.x + pAgent.head.normalized.y * toTarget.normalized.y;
        const float coefficient = 0.5f;//角速度
        return (dot - 1f) * (-coefficient);
    }
    /// <summary>
    /// 逃避
    /// </summary>
    /// <param name="pursuer"></param>
    /// <returns></returns>
    public Vector2 Evade(Vehicle pursuer)
    {
        Vector2 ToPursuer = pursuer.position - m_vehicle.position;
        float LookAheadTime = ToPursuer.magnitude / (m_vehicle.m_MaxSpeed + pursuer.velocity.magnitude);
        Debug.Log(pursuer.position + pursuer.velocity * LookAheadTime);
        return Flee(pursuer.position + pursuer.velocity * LookAheadTime);
    }
    /// <summary>
    /// 徘徊
    /// </summary>
    /// <returns></returns>
    public Vector2 Wander()
    {
        //timer++;
        //if (timer % 20 != 0) 
        //{
        //    return Seek(FromLocalToWorld(m_vehicle.wanderTarget.transform.position, m_vehicle.posotion, m_vehicle.head, m_vehicle.left));
        //}
        //timer = 0;
        m_WanderTarget += new Vector2(Random.Range(-1f, 1f) * m_WanderJitter, Random.Range(-1f, 1f) * m_WanderJitter);
        m_WanderTarget.Normalize();
        Debug.Log("local_1:(" + m_WanderTarget.x + "," + m_WanderTarget.y + ")");
        m_WanderTarget *= m_WanderRadius;
        Debug.Log("local_2:(" + m_WanderTarget.x + "," + m_WanderTarget.y + ")");
        m_vehicle.wanderTarget.transform.SetPositionAndRotation(FromLocalToWorld(m_WanderTarget + new Vector2(0, m_WanderDistance), m_vehicle.position, m_vehicle.head, m_vehicle.right), Quaternion.identity);
        return Seek(FromLocalToWorld(m_vehicle.wanderTarget.transform.position, m_vehicle.position, m_vehicle.head, m_vehicle.right));
    }
    /// <summary>
    /// 避开障碍
    /// </summary>
    /// <param name="pAgent"></param>
    /// <returns></returns>
    public Vector2 ObstacleAvoidance(Vehicle pAgent)
    {
        float move = Random.Range(-0.5f, 0.5f);
        float length = timeForAvoid * (1 + pAgent.m_MaxSpeed);
        Vector2 localOrigin = new Vector2();
        localOrigin.x = pAgent.size.x * move;
        localOrigin.y = -pAgent.size.y * 0.5f;
        float minDistance = 100f;
        int min_temp = -1;
        RaycastHit2D[] raycastHit2Ds = Physics2D.RaycastAll(FromLocalToWorld(localOrigin, pAgent.position, pAgent.head, pAgent.right), pAgent.head, length);
        for (int i = 0; i < raycastHit2Ds.Length; i++)
        {
            if (raycastHit2Ds[i].collider.tag == "Obstacle" && Vector2.Distance(raycastHit2Ds[i].collider.transform.position, pAgent.position) < minDistance) 
            {
                Debug.Log(raycastHit2Ds[i].collider.name);
                minDistance = Vector2.Distance(raycastHit2Ds[i].collider.transform.position, pAgent.position);
                min_temp = i;
            }
        }
        if (min_temp==-1)
        {
            return new Vector2(0, 0);
        }
        Vector2 toObstacle = (Vector2)raycastHit2Ds[min_temp].collider.transform.position - pAgent.position;
        float distance_len = Mathf.Abs(pAgent.head.x * toObstacle.x + pAgent.head.y * toObstacle.y);
        float distance_high = Mathf.Sqrt(minDistance * minDistance - distance_len * distance_len);
        float multiplier = (1f + (length - distance_len) / length) / 2;//[0,1]
        Vector2 steeringForce = new Vector2();//局部坐标
        float y_norm = (raycastHit2Ds[min_temp].collider.bounds.extents.x + pAgent.size.x / 2 - distance_high) / (raycastHit2Ds[min_temp].collider.bounds.extents.x + pAgent.size.x / 2);//[0,1]
        steeringForce.y = -y_norm * multiplier * pAgent.m_MaxForce;//[0,maxForce]
        float breakingWeight = 0.2f;
        steeringForce.x = (length - distance_len) / length * breakingWeight * m_vehicle.m_MaxForce;
        return FromLocalToWorld(steeringForce, new Vector2(0, 0), m_vehicle.head, m_vehicle.right);
    }
    /// <summary>
    /// 避开墙
    /// </summary>
    /// <returns></returns>
    public Vector2 WallAvoidance()
    {
        float length = timeForAvoid * (1 + m_vehicle.m_MaxSpeed);
        float angle = Random.Range(0.8f, 2.4f) + m_vehicle.rotation_z / 180 * 3.14f;
        RaycastHit2D[] raycastHit2Ds = Physics2D.RaycastAll(FromLocalToWorld(new Vector2(0, 0), m_vehicle.position, m_vehicle.head, m_vehicle.right), new Vector2(Mathf.Cos(angle), Mathf.Sin(angle)), length);
        float minDistance = 100f;
        int min_temp = -1;
        for (int i = 0; i < raycastHit2Ds.Length; i++)
        {
            if (raycastHit2Ds[i].collider.tag == "Wall")
            {
                Vector2 toCollider = (Vector2)raycastHit2Ds[i].collider.transform.position - m_vehicle.position;
                Vector2 ray = new Vector2(Mathf.Cos(angle) * length, Mathf.Sin(angle) * length);
                float deep = Mathf.Abs(Vector2.Dot(ray, raycastHit2Ds[i].collider.transform.up)) - Mathf.Abs(Vector2.Dot(toCollider, raycastHit2Ds[i].collider.transform.up));
                if (deep < minDistance) 
                {
                    minDistance = deep;
                    min_temp = i;
                }
            }
        }
        if (min_temp == -1) 
        {
            return new Vector2(0, 0);
        }
        Vector2 toWall = (Vector2)raycastHit2Ds[min_temp].collider.transform.position - m_vehicle.position;
        float temp = Vector2.Dot(toWall, raycastHit2Ds[min_temp].collider.transform.up);
        if (temp <= 0)   
        {
            return raycastHit2Ds[min_temp].collider.transform.up * (timeForAvoid - minDistance);
        }
        else
        {
            return -raycastHit2Ds[min_temp].collider.transform.up * (timeForAvoid - minDistance);
        }
    }
    /// <summary>
    /// 插入
    /// </summary>
    /// <param name="AgentA"></param>
    /// <param name="AgentB"></param>
    /// <returns></returns>
    public Vector2 Interpose(Vehicle AgentA, Vehicle AgentB)
    {
        Vector2 midPoint = (AgentA.position + AgentB.position) / 2.0f;
        float timeToReachMidPoint = Vector2.Distance(m_vehicle.position, midPoint) / m_vehicle.m_MaxSpeed;
        Vector2 APos = AgentA.position + AgentA.velocity * timeToReachMidPoint;
        Vector2 BPos = AgentB.position + AgentB.velocity * timeToReachMidPoint;
        midPoint = (APos + BPos) / 2.0f;
        return Arrive(midPoint, Deceleration.fast);
    }
    /// <summary>
    /// 视野内的collider
    /// </summary>
    /// <param name="Agent"></param>
    /// <returns></returns>
    public List<Collider2D> sight(Vehicle Agent)
    {
        Collider2D[] inSight = Physics2D.OverlapCircleAll(Agent.position, Agent.sightRadius);
        List<Collider2D> return_sight = new List<Collider2D>();
        for (int i = 0; i < inSight.Length; i++)
        {
            //确保在视角内
            Vector2 toCollider = (Vector2)inSight[i].transform.position - Agent.position;
            if (inSight[i].gameObject.name != Agent.name&& Vector2.Dot(toCollider.normalized, Agent.head) >= Mathf.Cos(Agent.sightAngle / 2))
            {
                //TODO:优化不一定连线上有东西就被完全挡住了
                RaycastHit2D[] between = Physics2D.RaycastAll(Agent.position, toCollider.normalized, Vector2.Distance(Agent.position, inSight[i].gameObject.transform.position));
                if (between.Length < 3) 
                {
                    return_sight.Add(inSight[i]);
                }
            }
        }
        return return_sight;
    }
    /// <summary>
    /// 判断目标（Vehicle）是否在视野范围内
    /// </summary>
    /// <param name="target"></param>
    /// <param name="Agent"></param>
    /// <returns></returns>
    public bool targetInsight(Vehicle target, Vehicle Agent)
    {
        List<Collider2D> inSight = sight(Agent);
        for (int i = 0; i < inSight.Count; i++)
        {
            if (inSight[i].gameObject.name == target.name)
            {
                return true;
            }
        }
        return false;
    }
    /// <summary>
    /// 计算隐藏点
    /// </summary>
    /// <param name = "posOb" ></ param >
    /// < param name="radiusOb"></param>
    /// <param name = "posTarget" ></ param >
    /// < returns ></ returns >
    public Vector2 GetHidingPoisition(Vector2 posOb,float radiusOb, Vector2 posTarget)
    {
        float distanceFromBoundary = 1f;
        float distAway = distanceFromBoundary + radiusOb;
        Vector2 toOb = posOb - posTarget;
        return (toOb.normalized * distAway) + posOb;
    }
    /// <summary>
    /// 隐藏
    /// </summary>
    /// <param name="target"></param>
    /// <returns></returns>
    public Vector2 Hide(Vehicle target)
    {
        if (Vector2.Distance(target.position, m_vehicle.position) > m_vehicle.PanicDistance)  
        {
            return new Vector2(0, 0);
        }
        //TODO:时间因素
        //确实要加时间因素，因为不加的话，当走出视野区域后就不再动了
        //TODO:如果周围有多个敌人呢？分别调用Hide函数是否会引起抖动？或者不能找到最优解？把target改成数组
        if (!targetInsight(target, m_vehicle)) 
        {
            return new Vector2(0, 0);
        }
        if (targetInsight(m_vehicle, target))
        {
            return Evade(target);
        }
        float radius = 6;
        Collider2D[] obstacles = Physics2D.OverlapCircleAll(m_vehicle.position, radius);
        float minDistance = 100;
        int min_temp = -1;
        List<Collider2D> targetSight = sight(target);
        for (int i = 0; i < obstacles.Length; i++)
        {
            float distance = Vector2.Distance(GetHidingPoisition(obstacles[i].gameObject.transform.position, obstacles[i].bounds.extents.x, target.position), m_vehicle.position);
            // 不能躲在target视野的障碍物
            bool inTargetSight = false;
            for (int j = 0; j < targetSight.Count; j++)
            {
                if (targetSight[j].gameObject.name == obstacles[i].gameObject.name) 
                {
                    inTargetSight = true;
                    break;
                }
            }
            if (inTargetSight==true)
            {
                continue;
            }
            if (obstacles[i].gameObject.tag != "Player" && distance < minDistance) 
            {
                minDistance = distance;
                min_temp = i;
            }
        }
        if (min_temp == -1) 
        {
            //HACK:这里的逃避只是个瞬时动作，不能持续（调转之后，不在视线内，返回（0，0）
            return Evade(target);
        }
        return Arrive(GetHidingPoisition(obstacles[min_temp].gameObject.transform.position, obstacles[min_temp].bounds.extents.x, target.position), Deceleration.fast);
        //TODO：在Arrive的时候绕target背后
    }
    /// <summary>
    /// 路径跟随
    /// </summary>
    /// <returns></returns>
    public Vector2 FollowPath()
    {
        if (m_Path.Count != 0) 
        {
            if (Vector2.Distance(m_Path[0], m_vehicle.position) < 0.01f)  
            {
                m_Path.RemoveAt(0);
            }
            return Arrive(m_Path[0], Deceleration.fast);
        }
        return new Vector2(0, 0);
    }
    /// <summary>
    /// 保持一定距离的追逐  
    /// </summary>
    /// <param name="leader"></param>
    /// <param name="offset"></param>
    /// <returns></returns>
    public Vector2 OffsetPursuit(Vehicle leader, Vector2 offset)
    {
        Vector2 worldOffsetPos = FromLocalToWorld(offset, leader.position, leader.head, leader.right);
        Vector2 toOffset = worldOffsetPos - m_vehicle.position;
        float lookAheadTime = toOffset.magnitude / m_vehicle.m_MaxSpeed * leader.velocity.magnitude;
        return Arrive(worldOffsetPos + leader.velocity * lookAheadTime, Deceleration.fast);
    }
    /// <summary>
    /// 分离
    /// </summary>
    /// <param name="neighbors"></param>
    /// <returns></returns>
    public Vector2 Sepraration(List<Collider2D> neighbors)
    {
        Vector2 SteeringForce = new Vector2(0, 0);
        int count = neighbors.Count;
        for (int i = 0; i < count; i++)
        {
            if (neighbors[i].gameObject.name != m_vehicle.name) 
            {
                Vector2 toAgent = m_vehicle.position - (Vector2)neighbors[i].transform.position;
                SteeringForce += toAgent.normalized / toAgent.magnitude;//HACK:
            }
        }
        return SteeringForce;
    }
    /// <summary>
    /// 队列
    /// </summary>
    /// <param name="neighbors"></param>
    /// <returns></returns>
    public Vector2 LineAlignment(List<Collider2D> neighbors)
    {
        Vector2 AverageHeading = new Vector2(0, 0);
        int count = neighbors.Count;
        for (int i = 0; i < count; i++)
        {
            if (neighbors[i].gameObject.name != m_vehicle.name) 
            {
                AverageHeading += (Vector2)neighbors[i].gameObject.transform.up;
            }
        }
        if (count != 0) 
        {
            AverageHeading /= (float)count;
            AverageHeading -= m_vehicle.head;
        }
        return AverageHeading;
    }
    /// <summary>
    /// 聚集
    /// </summary>
    /// <param name="neighbors"></param>
    /// <returns></returns>
    public Vector2 Cohesion(List<Collider2D> neighbors)
    {
        Vector2 centerOfMass = new Vector2(0, 0);
        Vector2 steeringForce = new Vector2(0, 0);
        int count = neighbors.Count;
        for (int i = 0; i < count; i++)
        {
            centerOfMass += (Vector2)neighbors[i].gameObject.transform.position;
        }
        if (count != 0) 
        {
            centerOfMass /= (float)count;
            steeringForce = Seek(centerOfMass);
        }
        return steeringForce;
    }
}