using UnityEngine;
using System.Collections;
//引入库
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Xml;
using System.Threading;
using System.IO;
using System;


public class UdpServer : MonoBehaviour
{
    //以下默认都是私有的成员
    Socket socket; //目标socket
    EndPoint clientEnd; //客户端
    IPEndPoint ipEnd; //侦听端口
    public string recvStr; //接收的字符串
    string sendStr; //发送的字符串
    public byte[] recvData = new byte[1024]; //接收的数据，必须为字节
    byte[] sendData = new byte[1024]; //发送的数据，必须为字节
    int recvLen; //接收的数据长度
    Thread connectThread; //连接线程
 
    
    string[] datastr;
    public RecDataCache datacache;
    int tracker_num = 6;

    //初始化
    public void InitSocket()
    {
        //定义侦听端口,侦听任何IP
        ipEnd = new IPEndPoint(IPAddress.Any, 4321);
        //定义套接字类型,在主线程中定义
        socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
        //服务端需要绑定ip
        socket.Bind(ipEnd);
        //定义客户端
        IPEndPoint sender = new IPEndPoint(IPAddress.Any, 0);
        clientEnd = (EndPoint)sender;
        //print("waiting for UDP dgram");

        //开启一个线程连接，必须的，否则主线程卡死
        // 用Loom的方法调用一个线程
        Loom.RunAsync(
            () =>
            {
                connectThread = new Thread(new ThreadStart(SocketReceive));
                connectThread.Start();
            }
            );
    }

    //void SocketSend(string sendStr)
    //{
    //    //清空发送缓存
    //    sendData = new byte[1024];
    //    //数据类型转换
    //    sendData = Encoding.ASCII.GetBytes(sendStr);
    //    //发送给指定客户端
    //    socket.SendTo(sendData, sendData.Length, SocketFlags.None, clientEnd);
    //}

    //服务器接收
    void SocketReceive()
    {
        //DateTime dt1 = System.DateTime.Now;
        //进入接收循环
        while (true)
        {
            //DateTime dt2 = System.DateTime.Now;
            //TimeSpan ts = dt2.Subtract(dt1);
            //Debug.Log(ts.TotalMilliseconds);
            //dt1 = dt2;
            //对data清零
            recvData = new byte[1024];
            //获取客户端，获取客户端数据，用引用给客户端赋值
            recvLen = socket.ReceiveFrom(recvData, ref clientEnd);
            //print("message from: " + clientEnd.ToString()); //打印客户端信息
            //输出接收到的数据
            recvStr = Encoding.ASCII.GetString(recvData, 0, recvLen);
            //print(recvStr);
            //Debug.Log(recvStr);
            //for (int i = 0; i<42;i++ )
            //{
            //    data[i] = float.Parse(recvStr.Split(' ')[i]);
            //}

            datastr = recvStr.Split(' ');
            //Debug.Log(datastr.Length);

            if (datastr.Length != (tracker_num*7+1))
                continue;


            //主线程中执行
            Loom.QueueOnMainThread((param) =>
            {
                for (int i=0;i<tracker_num;i++)
                {
                    datacache.pt_bvh[i] = new Vector3(float.Parse(datastr[i * 7 + 4]), float.Parse(datastr[i * 7 + 5]), float.Parse(datastr[i * 7 + 6]));
                    datacache.qt_bvh[i].w = float.Parse(datastr[i * 7]); datacache.qt_bvh[i].x = float.Parse(datastr[i * 7 + 1]); datacache.qt_bvh[i].y = float.Parse(datastr[i * 7 + 2]); datacache.qt_bvh[i].z = float.Parse(datastr[i * 7 + 3]);
                }
            }, null);
            
            
            //print("我是服务器，接收到客户端的数据" + recvStr);
            //将接收到的数据经过处理再发送出去
            //sendStr = "From Server: " + recvStr;
            //SocketSend(sendStr);
        }
    }

    //连接关闭
    void SocketQuit()
    {
        //关闭线程
        if (connectThread != null)
        {
            connectThread.Interrupt();
            connectThread.Abort();
        }
        //最后关闭socket
        if (socket != null)
            socket.Close();
        print("disconnect");
    }

    // Use this for initialization
    void Start()
    {
        InitSocket(); //在这里初始化server

    }
    void OnApplicationQuit()
    {
        SocketQuit();
    }
}