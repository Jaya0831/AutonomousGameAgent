using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VehicleManager : MonoBehaviour
{
    public static Dictionary<int, Vehicle> entityMap = new Dictionary<int, Vehicle>();
    /// <summary>
    /// 注册数据
    /// </summary>
    /// <param name="newEntity"></param>
    public static void RegisterVehicle(Vehicle vehicle)
    {
        entityMap.Add(vehicle.m_ID, vehicle);
    }
    /// <summary>
    /// 通过ID获得实例
    /// </summary>
    /// <param name="id"></param>
    /// <returns></returns>
    public static Vehicle GetVehicleFromID(int id)
    {
        Vehicle vehicle;
        entityMap.TryGetValue(id, out vehicle);
        return vehicle;
    }
    /// <summary>
    /// 删除数据
    /// </summary>
    /// <param name="entity"></param>
    public static void RemoveVehicle(Vehicle vehicle)
    {
        entityMap.Remove(vehicle.m_ID);
    }
}
