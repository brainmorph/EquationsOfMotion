
using System.Net;
using System.Net.Sockets;
using System.Text;

namespace EquationsOfMotion
{
    public class UdpComms
    {
        public UdpClient udpClient = new UdpClient();

        public UdpComms()
        {
            string host = "127.0.0.1";
            int port = 7777;

            Console.WriteLine($"Sending UDP to {host}:{port}");
        }

        public void Send(byte[] data)
        {
            string host = "127.0.0.1";
            int port = 7777;

            var remote = new IPEndPoint(IPAddress.Parse(host), port);

            //byte[] data = Encoding.UTF8.GetBytes("lol this rocks my socks.");
            udpClient.Send(data, remote);
        }
    }
}