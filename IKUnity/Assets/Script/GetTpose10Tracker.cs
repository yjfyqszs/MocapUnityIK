using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Xml;
using System.IO;
using System.Text;

public class GetTpose10Tracker : MonoBehaviour
{
    public Quaternion[] T_R = new Quaternion[10];//����ֱ�Ӹ�Transform��ֵ�������м��һֱ����
    public Vector3[] T_P = new Vector3[10];
    public GameObject[] Joint = new GameObject[10];
    //public GameObject[] All_T = new GameObject[21];

    // Start is called before the first frame update
    void Start()
    {
        
    }

    void Awake()//��ȡ��ʼT_pose
    {
        for (int i = 0; i < 10; i++)
        {
            T_R[i] = Joint[i].GetComponent<Transform>().rotation;
            T_P[i] = Joint[i].GetComponent<Transform>().position;
            //Debug.Log(T_R[i]);
        }

        ////д��Tracker��ʼ״̬�����ڻ�ȡ��������
        //FileStream fs = new FileStream("Assets/Chracter_T.txt", FileMode.Create);//�ı����벻����
        //StreamWriter wr = new StreamWriter(fs);
        //string oneline = "";
        //for (int i = 0; i < 21; i++)
        //{
        //    Vector3 p = All_T[i].GetComponent<Transform>().position;
        //    Quaternion q = All_T[i].GetComponent<Transform>().rotation;//��Ҫ���±곯��
        //    oneline += Convert.ToString(p.x*100) + " " + Convert.ToString(p.y*100) + " " + Convert.ToString(p.z*100) + " "
        //        + Convert.ToString(q.w) + " " + Convert.ToString(q.x) + " " + Convert.ToString(q.y) + " " + Convert.ToString(q.z) + " ";//����cm
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
