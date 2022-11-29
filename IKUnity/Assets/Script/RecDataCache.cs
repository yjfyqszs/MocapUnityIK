using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RecDataCache : MonoBehaviour
{
    //临时存储UDP接收数据
    public Vector3[] pt_bvh=new Vector3[6];
    public Quaternion[] qt_bvh = new Quaternion[6];
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        //Debug.Log(pt_bvh[0]);
    }
}
