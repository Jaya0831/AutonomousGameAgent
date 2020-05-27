using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraController : MonoBehaviour
{
    public float speed = 4;
    public float mousespeed = 150;
    //public GameObject background;
    // Update is called once per frame
    private void Awake()
    {
        //background.transform.SetPositionAndRotation(new Vector3(transform.position.x, transform.position.y, background.transform.position.z), Quaternion.identity);
    }
    void Update()
    {
        //float h = Input.GetAxis("Horizontal");
        //float v = Input.GetAxis("Vertical");
        //transform.Translate(new Vector3(h, v, 0) * speed * Time.deltaTime, Space.World);
        //background.transform.SetPositionAndRotation(new Vector3(transform.position.x, transform.position.y, background.transform.position.z), Quaternion.identity);
    }
}
