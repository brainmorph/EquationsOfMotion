namespace EquationsOfMotion
{
    internal class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("Hello, World!");

            EOM eom = new EOM();
            eom.Step();

            UdpComms udp = new UdpComms();






            double[] values = new double[] { 1.1, 2.2, 3.3 };
            byte[] data = new byte[values.Length * sizeof(double)];

            Buffer.BlockCopy(values, 0, data, 0, data.Length);

            udp.Send(data);
        }
    }
}
