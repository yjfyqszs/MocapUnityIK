using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System;
using System.Xml;
using System.IO;
using System.Text;

public class SetPoseUDP : MonoBehaviour
{// public GameObject[] Joint = new GameObject[6];
    public GameObject[] Tracker = new GameObject[6];
    public GetTpose T_pose;
    //public Transform[] T_H = new Transform[6];
    public Quaternion[] Calib_q = new Quaternion[6];
    //public Transform[] Calib_H = new Transform[6];//不能直接设置为transform
    public Quaternion[] Calib_Hq = new Quaternion[6];
    public Vector3[] Calib_Hp = new Vector3[6];

    public Vector3[] pt_bvh = new Vector3[6];//真实数据
    public Quaternion[] qt_bvh = new Quaternion[6];//真实数据
    //public GameObject[] OutObj = new GameObject[21];


    Vector3 scale = new Vector3(1.0f, 1.0f, 1.0f);
    public Vector3[] Tracker_Tp = new Vector3[6];//Tracker_T0
    public Vector3[] Joint_Tp = new Vector3[6];//Joint_T0

    public float[,] bvh_data = new float[4502, 42];//读取BVH数据
    public string[] datastr = new string[42];
    float S_sum = 0;
    int tracker_num = 6;
    public int frame_index = 0;
    public RecDataCache datacache;

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {

        for (int i = 0; i < tracker_num;i++ )
        {
            pt_bvh[i] = datacache.pt_bvh[i];
            qt_bvh[i] = datacache.qt_bvh[i];
        }

        ////标定方法1
        //if (Input.GetKeyDown(KeyCode.A))
        //{
        //    S_sum = 0;
        //    Debug.Log("按键标定");
        //    for (int i = 0; i < tracker_num; i++)
        //    {
        //        Calib_q[i] = qt_bvh[i] * T_pose.T_R[i];//朝向
        //        float s = T_pose.T_P[i].magnitude / pt_bvh[i].magnitude;
        //        S_sum += s;
        //    }
        //    //Debug.Log(T_pose.T_R[0]);//T_H[0]的数值会不一样？？？
        //    S_sum /= tracker_num;//平均缩放因子
        //    //Debug.Log(S_sum);
        //    for (int i = 0; i < tracker_num; i++)//标位置
        //    {
        //        Vector3 e1 = pt_bvh[i].normalized;
        //        Vector3 e2 = T_pose.T_P[i].normalized;
        //        float thda = Mathf.Acos(Vector3.Dot(e1, e2)) * Mathf.Rad2Deg;//e1到e2的夹角
        //        Vector3 en = Vector3.Cross(e1, e2).normalized;//Cross为e1Xe2
        //        Quaternion q0 = Quaternion.AngleAxis(thda, en);//e2=q0*e1
        //        //q0 = Quaternion.Inverse(q0);

        //        Vector3 sP = S_sum * pt_bvh[i];
        //        Vector3 sRP = q0 * sP;
        //        Vector3 ts = T_pose.T_P[i] - sRP;
        //        Calib_Hp[i] = ts;
        //        Calib_Hq[i] = q0;
        //    }
        //}


        //标定方法2
        if (Input.GetKeyDown(KeyCode.A))
        {
            Debug.Log("按键标定");
            for (int i = 0; i < tracker_num; i++)
            {
                Calib_q[i] = qt_bvh[i] * T_pose.T_R[i];//朝向   
                Tracker_Tp[i] = pt_bvh[i];
            }

            scale.x = (T_pose.T_P[3].x - T_pose.T_P[2].x) / (pt_bvh[3].x - pt_bvh[2].x);
            scale.y = (2 * T_pose.T_P[1].y - T_pose.T_P[4].y - T_pose.T_P[5].y) / (2 * pt_bvh[1].y - pt_bvh[4].y - pt_bvh[5].y);
        }


        //实时更新位姿
        for (int i = 0; i < tracker_num; i++)
        {
            Tracker[i].GetComponent<Transform>().rotation = Quaternion.Inverse(qt_bvh[i]) * Calib_q[i];

            ////标定方法1对应更新
            //Vector3 linshi1 = S_sum * pt_bvh[i];
            //Vector3 linshi = Calib_Hq[i] * linshi1;
            //Tracker[i].GetComponent<Transform>().position = linshi + Calib_Hp[i];

            //标定方法2对应更新
            if (frame_index < 100)//都是T-pose
            {
                Tracker[i].GetComponent<Transform>().position = new Vector3(pt_bvh[i].x * scale.x, pt_bvh[i].y * scale.y, pt_bvh[i].z * scale.z);
                Joint_Tp[i] = new Vector3(pt_bvh[i].x * scale.x, pt_bvh[i].y * scale.y, pt_bvh[i].z * scale.z);
            }
            else
            {
                Tracker[i].GetComponent<Transform>().position = Joint_Tp[i] + new Vector3((pt_bvh[i].x - Tracker_Tp[i].x) * scale.x, (pt_bvh[i].y - Tracker_Tp[i].y) * scale.y, (pt_bvh[i].z - Tracker_Tp[i].z) * scale.z);
            }

        }

        System.Threading.Thread.Sleep(20);//延时
        frame_index++;
    }
}
