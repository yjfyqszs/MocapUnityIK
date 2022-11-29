using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System;
using System.Xml;
using System.IO;
using System.Text;

public class SetPose : MonoBehaviour
{
   // public GameObject[] Joint = new GameObject[6];
    public GameObject[] Tracker = new GameObject[6];
    public GetTpose T_pose;
    //public Transform[] T_H = new Transform[6];
    public Quaternion[] Calib_q = new Quaternion[6];
    //public Transform[] Calib_H = new Transform[6];//不能直接设置为transform
    public Quaternion[] Calib_Hq = new Quaternion[6];
    public Vector3[] Calib_Hp = new Vector3[6];

    public Vector3[] pt_bvh=new Vector3[6];//真实数据
    public Quaternion[] qt_bvh=new Quaternion[6];//真实数据
    //public GameObject[] OutObj = new GameObject[21];


    Vector3 scale = new Vector3(1.0f, 1.0f, 1.0f);
    public Vector3[] Tracker_Tp = new Vector3[6];//Tracker_T0
    public Vector3[] Joint_Tp = new Vector3[6];//Joint_T0

    public float[,] bvh_data = new float[4502,42];//读取BVH数据
    public string[] datastr = new string[42];
    float S_sum = 0;
    int tracker_num = 6;
    public int frame_index = 0;


    // Start is called before the first frame update
    void Start()
    {
        //读取BVH数据
        FileStream fs = new FileStream("Assets/SendUnity6TrackerDance.txt", FileMode.Open, FileAccess.Read);
        StreamReader sr = new StreamReader(fs, System.Text.Encoding.Default);
        string str;
        for (int i=0;(str=sr.ReadLine())!=null;i++)
        {
            datastr = str.Split(' ');
            //Debug.Log(datastr.Length);
            for (int j = 0; j < tracker_num * 7; j++)
            {
                bvh_data[i, j] = float.Parse(datastr[j]);
            }
        }
        sr.Close();
        //Debug.Log(bvh_data[0,0]);
    }

    // Update is called once per frame
    void Update()
    {
        //txt数据
        if (frame_index >= 4502) frame_index = 0;
        for (int i = 0; i < tracker_num; i++)
        {
            Vector3 p = new Vector3(bvh_data[frame_index,i*7+4], bvh_data[frame_index, i * 7 + 5], bvh_data[frame_index, i * 7 + 6]);
            Quaternion q;
            q.w = bvh_data[frame_index, i * 7]; q.x = bvh_data[frame_index, i * 7 + 1]; q.y = bvh_data[frame_index, i * 7 + 2]; q.z = bvh_data[frame_index, i * 7 + 3];
            //Debug.Log(pt_bvh[i]);
            pt_bvh[i] = p;
            qt_bvh[i] = q;

        }
        frame_index++;

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
         for (int i=0;i<tracker_num;i++)
        {
            Tracker[i].GetComponent<Transform>().rotation = Quaternion.Inverse(qt_bvh[i]) * Calib_q[i];

            ////标定方法1对应更新
            //Vector3 linshi1 = S_sum * pt_bvh[i];
            //Vector3 linshi = Calib_Hq[i] * linshi1;
            //Tracker[i].GetComponent<Transform>().position = linshi + Calib_Hp[i];

            //标定方法2对应更新
            if (frame_index < 300)//都是T-pose
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
    }

    void LateUpdate()
    {
        //FileStream fs = new FileStream("Assets/Output.txt", FileMode.Append);//文本加入不覆盖
        //StreamWriter wr = new StreamWriter(fs);
        //string oneline = Convert.ToString(frame_index) + " ";
        //for (int i = 0; i < 21; i++)
        //{
        //    Vector3 p = OutObj[i].GetComponent<Transform>().position;
        //    Quaternion q = OutObj[i].GetComponent<Transform>().rotation;//需要重新标朝向
        //    oneline += Convert.ToString(p.x) + " " + Convert.ToString(p.y) + " " + Convert.ToString(p.z) + " "
        //        + Convert.ToString(q.w) + " " + Convert.ToString(q.x) + " " + Convert.ToString(q.y) + " " + Convert.ToString(q.z) + " ";
        //}
        //wr.WriteLine(oneline);
        //wr.Flush();
        //wr.Close();
        //fs.Close();
    }
}
