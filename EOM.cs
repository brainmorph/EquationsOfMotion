using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace EquationsOfMotion
{
    public class EOM
    {
        public State CurrentState = new State();
        public State NextState = new State();

        public double g = 9.81;         // m/s^2
        const double dt = 0.016667;     // seconds

        public void Step()
        {
            CalculateLinearDynamics(CurrentState);
            CalculateRotationalDynamics();
        }

        public void CalculateLinearDynamics(State s)
        {
            NextState = s; // Starting with current state

            // Given CurrentState, find NextState by finding the derivative of each state variable
            double F_Tx = 7_000;    // Newtons (assume cessna 172 engine)
            double F_Gx = -s.m * g * Math.Sin(s.Theta); // Newtons
            double alpha = CalculateAlpha(s);
            
            double D = CalculateDrag(alpha, s);
            double L = CalculateLift(alpha, s);
            double F_Ax = -D * Math.Cos(alpha) + L * Math.Sin(alpha);     // Newtons
            double F_x = F_Gx + F_Ax + F_Tx;
            double U_dot = F_x / s.m - s.Q * s.W + s.R * s.V;
            
            double F_Ty = 0;        // Newtons
            double F_Gy = s.m * g * Math.Sin(s.Phi) * Math.Cos(s.Theta); // Newtons
            double beta = CalculateBeta(s);
            double F_Ay = CalculateSideForce(beta, s);        // Newtons (assume no side slip)
            double F_y = F_Gy + F_Ay + F_Ty;
            double V_dot = F_y / s.m - s.R * s.U + s.P * s.W;

            double F_Tz = 0;        // Newtons
            double F_Gz = s.m * g * Math.Cos(s.Phi) * Math.Cos(s.Theta);   // Newtons
            double F_Az = 9_300;    // Newtons (assume cruise and 0 alpha)
            double F_z = F_Gz + F_Az + F_Tz;
            double W_dot = F_z / s.m - s.P * s.V + s.Q * s.U;

            // Advance to next state
            NextState.U = U_dot * dt + s.U;
            NextState.V = V_dot * dt + s.V;
            NextState.W = W_dot * dt + s.W;
            NextState.alpha = alpha;
            NextState.U_dot = U_dot;
            NextState.V_dot = V_dot;
            NextState.W_dot = W_dot;

            CurrentState = NextState;
        }

        public double CalculateAlpha(State s)
        {
            return Math.Atan2(s.W , s.U);
        }

        public double CalculateBeta(State s)
        {
            return Math.Atan2(s.V, Math.Sqrt(s.U * s.U + s.W * s.W));
        }

        public double CalculateDrag(double alpha, State s)
        {
            double rho = 1.06; // kg/m^3 (guesstimate)
            double Cd = C_D(alpha);
            double Vc = Math.Sqrt(s.U * s.U + s.V * s.V + s.W * s.W);   // m/s
            double surfaceArea = 16.2; // (also guesstimate)
            double d = 0.5 * rho * Vc * Vc * surfaceArea * Cd;
            return d;
        }

        public double CalculateLift(double alpha, State s)
        {
            double rho = 1.06; // kg/m^3 (guesstimate)
            double Cd = C_L(alpha);
            double Vc = Math.Sqrt(s.U * s.U + s.V * s.V + s.W * s.W);   // m/s
            double surfaceArea = 16.2; // (guesstimate)
            double l = 0.5 * rho * Vc * Vc * surfaceArea * Cd;
            return l;
        }

        public double CalculateSideForce(double beta, State s)
        {
            double rho = 1.06; // kg/m^3 (guesstimate)
            double Cy = C_Y(beta);
            double Vc = Math.Sqrt(s.U * s.U + s.V * s.V + s.W * s.W);   // m/s
            double surfaceArea = 16.2; // (guesstimate)
            double sf = 0.5 * rho * Vc * Vc * surfaceArea * Cy;
            return sf;
        }

        public double C_D(double alpha)
        {
            // assume cessna 172 rough values
            if (alpha <= 2)
            {
                return 0.045;
            }
            else if (alpha <= 4)
            {
                return 0.050;
            }
            else if (alpha <= 5)
            {
                return 0.055;
            }
            else if (alpha <= 8)
            {
                return 0.075;
            }
            else if (alpha <= 9)
            {
                return 0.090;
            }
            else if (alpha <= 16)
            {
                return 0.11;
            }

            return 0.12;
        }

        public double C_L(double alpha)
        {
            // assume cessna 172 rough values
            if (alpha <= 2)
            {
                return 0.35;
            }
            else if (alpha <= 4)
            {
                return 0.50;
            }
            else if (alpha <= 5)
            {
                return 0.60;
            }
            else if (alpha <= 6)
            {
                return 0.75;
            }
            else if (alpha <= 9)
            {
                return 1.0;
            }
            else if (alpha <= 16)
            {
                return 1.25;
            }

            return 1.3;
        }

        public double C_Y(double beta)
        {
            // assume cessna 172 rough values
            if (beta <= 1)
            {
                return 0.0;
            }
            else if (beta <= 2)
            {
                return 0.014;
            }
            else if (beta <= 3)
            {
                return 0.028;
            }
            else if (beta <= 5)
            {
                return 0.042;
            }
            else if (beta <= 6)
            {
                return 0.070;
            }
            else if (beta <= 10)
            {
                return 0.11;
            }
            
            return 0.14;
        }

        public void CalculateRotationalDynamics()
        {
            ComputeBodyFrameMomentsInStabilityAxes(CurrentState);
            ComputeBodyFrameAccelerations(0.0, 0.0, 0.0, CurrentState);
        }

        public void ComputeBodyFrameMomentsInStabilityAxes(State s)
        {
            double rho = 1.06; // kg/m^3 (guesstimate)
            double Vc = Math.Sqrt(s.U * s.U + s.V * s.V + s.W * s.W);   // m/s
            double surfaceArea = 16.2; // (guesstimate)
            double c_bar = 0;
            int C_m0 = 0;
            int C_m_alpha = 0;
            int alpha_W = 0;
            int C_m_de = 0;
            int de = 0; // elevator angle (radians)
            double C_mq = 0;
            double C_m_alpha_dot = 0;
            double M_stab = 0.5 * rho * Vc * Vc * surfaceArea * c_bar * (C_m0 + C_m_alpha * alpha_W + C_m_de * de) +
                            0.25 * rho * Vc * Vc * surfaceArea * c_bar * c_bar *
                            (C_mq * s.Q + C_m_alpha_dot * s.alpha_dot);
        }

        private void ComputeBodyFrameAccelerations(double L, double M, double N, State s)
        {
            // Compute body frame angular accelerations
            double P_dot = L + (s.I_yy - s.I_zz) * s.Q * s.R +
                           s.I_xz * (s.R_dot * s.P + s.P * s.Q) /
                           s.I_xx;

            double Q_dot = M + (s.I_zz - s.I_xx) * s.R * s.P +
                           s.I_xz * (s.R * s.R - s.P * s.P) /
                           s.I_yy;

            double R_dot = N + (s.I_xx - s.I_yy) * s.P * s.Q +
                           s.I_xz * (s.P_dot - s.Q * s.R) /
                           s.I_zz;

            s.P_dot = P_dot;
            s.Q_dot = Q_dot;
            s.R_dot = R_dot;

            s.P += P_dot * dt;
            s.Q += Q_dot * dt;
            s.R += R_dot * dt;
        }

    }
}
