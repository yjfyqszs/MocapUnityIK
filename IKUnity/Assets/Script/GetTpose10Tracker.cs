using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Xml;
using System.IO;
using System.Text;

public class GetTpose10Tracker : MonoBehaviour
{
    public Quaternion[] T_R = new Quaternion[10];//不能直接给Transform赋值！！！中间会一直错！！
    public Vector3[] T_P = new Vector3[10];
    public GameObject[] Joint = new GameObject[10];
    //public GameObject[] All_T = new GameObject[21];

    // Start is called before the first frame update
    void Start()
    {
        
    }

    void Awake()//获取初始T_pose
    {
        for (int i = 0; i < 10; i++)
        {
            T_R[i] = Joint[i].GetComponent<Transform>().rotation;
            T_P[i] = Joint[i].GetComponent<Transform>().position;
            //Debug.Log(T_R[i]);
        }

        ////写入Tracker初始状态，用于获取骨骼数据
        //FileStream fs = new FileStream("Assets/Chracter_T.txt", FileMode.Create);//文本加入不覆盖
        //StreamWriter wr = new StreamWriter(fs);
        //string oneline = "";
        //for (int i = 0; i < 21; i++)
        //{
        //    Vector3 p = All_T[i].GetComponent<Transform>().position;
        //    Quaternion q = All_T[i].GetComponent<Transform>().rotation;//需要重新标朝向
        //    oneline += Convert.ToString(p.x*100) + " " + Convert.ToString(p.y*100) + " " + Convert.ToString(p.z*100) + " "
        //        + Convert.ToString(q.w) + " " + Convert.ToString(q.x) + " " + Convert.ToString(q.y) + " " + Convert.ToString(q.z) + " ";//换成cm
        //}
        //wr.WriteLine(oneline);
        //wr.Flush();
        //wr.Close();
        //fs.Close();

    }
    // Update is called once per frame
    void Update()
    {
        
    }
}
