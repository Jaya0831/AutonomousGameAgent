using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class tesk : MonoBehaviour
{
    public float speed = 4;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        //float h = Input.GetAxis("Horizontal");
        //float v = Input.GetAxis("Vertical");
        //transform.Translate(new Vector3(h, v, 0) * speed * Time.deltaTime, Space.World);
        RaycastHit2D[] raycastHit2Ds = Physics2D.RaycastAll(new Vector2(-2.8f, -2.8f), new Vector2(1, 1), 8);
        for (int i = 0; i < raycastHit2Ds.Length; i++)
        {
            Debug.Log(raycastHit2Ds[i].collider.name);
        }
    }
    private void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawLine(new Vector3(-2.8f, -2.8f, -4f), new Vector3(2.8f, 2.8f, -4));
    }
}

