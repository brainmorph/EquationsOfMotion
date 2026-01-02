using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace EquationsOfMotion
{
    public class State
    {
        public double U { get; set; } = 36.0; // m/s
        public double V { get; set; } = 0;
        public double W { get; set; } = 0;
        public double P { get; set; } = 0;
        public double Q { get; set; } = 0;
        public double R { get; set; } = 0;

        public double m { get; set; } = 760;    // kilograms

        public double Theta { get; set; } = 0;
        public double Phi { get; set; } = 0;
        public double Psi { get; set; } = 0;

        public double U_dot { get; set; } = 0;
        public double V_dot { get; set; } = 0;
        public double W_dot { get; set; } = 0;
        public double P_dot { get; set; } = 0;
        public double Q_dot { get; set; } = 0;
        public double R_dot { get; set; } = 0;
        public double Theta_dot { get; set; } = 0;
        public double Phi_dot { get; set; } = 0;
        public double Psi_dot { get; set; } = 0;

        public double I_xx = 2424; // kg*m^2
        public double I_yy = 2427; // kg*m^2
        public double I_zz = 4372; // kg*m^2
        public double I_xy = 0;
        public double I_xz = -161; // kg*m^2
        public double I_yz = 0;
    }
}
