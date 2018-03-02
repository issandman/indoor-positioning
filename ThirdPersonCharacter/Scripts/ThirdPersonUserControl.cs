using System;
using UnityEngine;
using UnityStandardAssets.CrossPlatformInput;
using System.IO.Ports;
using System.Threading;

namespace UnityStandardAssets.Characters.ThirdPerson
{
    [RequireComponent(typeof (ThirdPersonCharacter))]
    public class ThirdPersonUserControl : MonoBehaviour
    {
        private ThirdPersonCharacter m_Character; // A reference to the ThirdPersonCharacter on the object
        private Transform m_Cam;                  // A reference to the main camera in the scenes transform
        private Vector3 m_CamForward;             // The current forward direction of the camera
        private Vector3 m_Move;
        private bool m_Jump;                      // the world-relative desired move direction, calculated from the camForward and user input.

        private Thread recvThread;

        int j = 0;
        int count = 0;
        float z = 0;
        float t = 0;
        float tt = 0;
        int temp = 0;

        float X = 0;
        float Y = 0;
        float y = 0;

        byte[] dataBytes = new byte[1];//存储数据
        byte[] dataBytes_ = new byte[1];
        byte[] dataBytesFloat = new byte[4]; //存储float数据
        char[] sbuf = new char[1024];

        public string portName = "COM6";
        public int baudRate = 9600;
        public Parity parity = Parity.None;
        public int dataBits = 8;
        public StopBits stopBits = StopBits.One;
        SerialPort sp = null;

        private void Start()
        {
            // get the transform of the main camera
            if (Camera.main != null)
            {
                m_Cam = Camera.main.transform;
            }
            else
            {
                Debug.LogWarning(
                    "Warning: no main camera found. Third person character needs a Camera tagged \"MainCamera\", for camera-relative controls.", gameObject);
                // we use self-relative controls in this case, which probably isn't what the user wants, but hey, we warned them!
            }

            // get the third person character ( this should never be null due to require component )
            m_Character = GetComponent<ThirdPersonCharacter>();

            OpenPort();
            recvThread = new Thread(DataReceiveFunction);
            recvThread.Start();

            for (byte i = 0; i < dataBytes.Length; i++)
            {
                dataBytes[i] = 0;
            }
        }

        //打开串口
        public void OpenPort()
        {
            sp = new SerialPort(portName, baudRate, parity, dataBits, stopBits);
            sp.ReadTimeout = 400;
            try
            {
                sp.Open();
                z = 100;
                t = 11;
            }
            catch (Exception ex)
            {
                Debug.Log(ex.Message);
            }
        }

        //关闭串口
        public void ClosePort()
        {
            try
            {
                sp.Close();
            }
            catch (Exception ex)
            {
                Debug.Log(ex.Message);
            }
        }

        void DataReceiveFunction()
        {
            char[] sbuf = new char[1024];
            try
            {
                Byte[] buf = new Byte[1];
                if (sp.IsOpen) { sp.Read(buf, 0, 1); t = 55; }

                if (buf.Length == 0)
                {
                    t = 22;
                    Debug.Log("Warning: Receive buffer is null.");
                }
                else if (buf[0] == 121 || buf[0] == 88 || buf[0] == 89)
                {//第一次找到数据头标志 
                    int i = 0;
                    t = 33;
                    while (true)
                    {//不断读取欧拉角                          

                        if (buf[0] == 121)//读y数据
                        { temp = 1; }
                        if (buf[0] == 88)//读X数据
                        { temp = 2; }
                        if (buf[0] == 89)//读Y数据
                        { temp = 3; }

                        sp.Read(buf, 0, 1);//读一个字节
                        t = 66;

                        if(buf[0] == 121 || buf[0] == 88 || buf[0] == 89)
                        {
                            t = 77;
                            String s = new String(sbuf, 0, i);
                            if(temp == 1)
                            {
                                y = float.Parse(s);
                                z = y;
                            }
                            if (temp == 2)
                            {
                                X = float.Parse(s);
                            }
                            if (temp == 3)
                            {
                                Y = float.Parse(s);
                            }
                            i = 0;
                            temp = 0;
                        }
                        else
                        {
                            //保存该字节
                            t = 88;
                            sbuf[i] = (char)buf[0];
                            i++;
                        }

                    }
                }
            }
            catch (Exception ex)
            {
                Debug.Log(ex);
            }
        }

        void OnApplicationQuit()
        {
            ClosePort();
        }


        private void Update()
        {
            if (!m_Jump)
            {
                m_Jump = CrossPlatformInputManager.GetButtonDown("Jump");
            }
        }


        // Fixed update is called in sync with physics
        private void FixedUpdate()
        {
            // read inputs
            float h = CrossPlatformInputManager.GetAxis("Horizontal");
            float v = CrossPlatformInputManager.GetAxis("Vertical");
            bool crouch = Input.GetKey(KeyCode.C);

            Y = 100;
            if (count == 80)
                Y = 200;

            transform.rotation = Quaternion.AngleAxis(y, Vector3.up);
            tt = X;
            // v = Y;

            // calculate move direction to pass to character
            if (m_Cam != null)
            {
                // calculate camera relative direction to move:
                m_CamForward = Vector3.Scale(m_Cam.forward, new Vector3(1, 0, 1)).normalized;
                m_Move = v * m_CamForward + h * m_Cam.right;
                if (X > 0)
                    //m_Move = v * m_CamForward + h * m_Cam.right;
                    //m_Move.z += X;
                    transform.Translate(0.0f, 0.0f, X * 5f * Time.deltaTime);
                else if (X < 0)
                {
                    transform.Translate(0.0f, 0.0f, X * 2 * Time.deltaTime);
                    tt -= 1;
                }
                j = 10;
            }
            else
            {
                // we use world-relative directions in the case of no main camera
                m_Move = v*Vector3.forward + h*Vector3.right;

                j = 5;
            }
#if !MOBILE_INPUT
			// walk speed multiplier
	        if (Input.GetKey(KeyCode.LeftShift)) m_Move *= 0.5f;
#endif

            // pass all parameters to the character control script
//            m_Character.speed(X);
            m_Character.Move(m_Move, crouch, m_Jump);
            m_Jump = false;

            if (count == 99) count = 0;
            count++;
        }

        void OnGUI()
        {
            GUI.Label(new Rect(20, 20, 100, 35), "The score: " + z + "open:" + tt + " !!" + t);
            GUI.color = Color.red;
        }
    }
}
