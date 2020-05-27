using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BaseGameEntity : MonoBehaviour
{
    /// <summary>
    /// 每个实体具有一个唯一的识别数字
    /// </summary>
    public int m_ID;
    //TODO:类型，位置，包围半径，缩放比例
    /// <summary>
    /// 下一个有效的ID，每次BaseGameEntity被实例化时更
    /// </summary>
    public static int m_NextValidID = 0;
    /// <summary>
    /// 生成id
    /// </summary>
    public void initID()
    {
        m_ID = m_NextValidID;
        m_NextValidID++;
    }

    public Vector2 position
    {
        get
        {
            return new Vector2(transform.position.x, transform.position.y);
        }
    }
    public Vector2 size
    {
        get
        {
            return new Vector2(GetComponent<BoxCollider2D>().size.x * transform.localScale.x, GetComponent<BoxCollider2D>().size.y * transform.localScale.y);
        }
    }

    
    /// <summary>
    /// 邻居容器（存储邻居的collider）
    /// </summary>
    public List<Collider2D> neighborTag = new List<Collider2D>();

    public void TagNeighbors(BaseGameEntity entity, float radius, float angle = Mathf.PI * 2)
    {
        neighborTag.Clear(); 
        Collider2D[] neighbors = Physics2D.OverlapCircleAll(entity.transform.position, radius);
        for (int i = 0; i < neighbors.Length; i++)
        {
            Vector2 toNeighbors = (Vector2)neighbors[i].transform.position - (Vector2)entity.transform.position;
            if (neighbors[i].gameObject.name != entity.name && neighbors[i].gameObject.tag == "Player" && Vector2.Dot(entity.transform.up, toNeighbors.normalized) > Mathf.Cos(angle / 2.0f))
            {
                neighborTag.Add(neighbors[i]);
            }
        }
    }

}
