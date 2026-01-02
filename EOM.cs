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

        public double g = 9.81;

        public EOM()
        {
            CalculateLinearDynamics();
            CalculateRotationalDynamics();
        }

        public void CalculateLinearDynamics()
        {
            // Given CurrentState, find NextState by finding the derivative of each state variable
            double F_Tx = 7_000;    // Newtons (assume cessna 172 engine)
            double F_Gx = -CurrentState.m * g * Math.Sin(CurrentState.Theta); // Newtons
            double alpha = CalculateAlpha(CurrentState);
            double D = CalculateDrag(alpha, CurrentState);
            double L = CalculateLift(alpha, CurrentState);
            double F_Ax = -D * Math.Cos(alpha) + L * Math.Sin(alpha);     // Newtons
            double F_x = F_Gx + F_Ax + F_Tx;
            double U_dot = F_x / CurrentState.m - CurrentState.Q * CurrentState.W + CurrentState.R * CurrentState.V;

            double F_Ty = 0;        // Newtons
            double F_Gy = CurrentState.m * g * Math.Sin(CurrentState.Phi) * Math.Cos(CurrentState.Theta); // Newtons
            double beta = CalculateBeta(CurrentState);
            double F_Ay = CalculateSideForce(beta, CurrentState);        // Newtons (assume no side slip)
            double F_y = F_Gy + F_Ay + F_Ty;
            double V_dot = F_y / CurrentState.m - CurrentState.R * CurrentState.U + CurrentState.P * CurrentState.W;

            double F_Tz = 0;        // Newtons
            double F_Gz = CurrentState.m * g * Math.Cos(CurrentState.Phi) * Math.Cos(CurrentState.Theta);   // Newtons
            double F_Az = 9_300;    // Newtons (assume cruise and 0 alpha)
            double F_z = F_Gz + F_Az + F_Tz;
            double W_dot = F_z / CurrentState.m - CurrentState.P * CurrentState.V + CurrentState.Q * CurrentState.U;

            // Advance to next state
            double dt = 0.0166;      // seconds
            NextState.U = U_dot * dt + CurrentState.U;
            NextState.V = V_dot * dt + CurrentState.V;
            NextState.W = W_dot * dt + CurrentState.W;

            CurrentState = NextState;
        }

        public void CalculateRotationalDynamics()
        {
            ComputeBodyFrameAccelerations(0.0, 0.0, 0.0);
        }

        private void ComputeBodyFrameAccelerations(double L, double M, double N)
        {
            // Compute body frame angular accelerations
            double P_dot = L + (CurrentState.I_yy - CurrentState.I_zz) * CurrentState.Q * CurrentState.R +
                           CurrentState.I_xz * (CurrentState.R_dot * CurrentState.P + CurrentState.P * CurrentState.Q) /
                           CurrentState.I_xx;

            double Q_dot = M + (CurrentState.I_zz - CurrentState.I_xx) * CurrentState.R * CurrentState.P +
                           CurrentState.I_xz * (CurrentState.R * CurrentState.R - CurrentState.P * CurrentState.P) /
                           CurrentState.I_yy;

            double R_dot = N + (CurrentState.I_xx - CurrentState.I_yy) * CurrentState.P * CurrentState.Q +
                           CurrentState.I_xz * (CurrentState.P_dot - CurrentState.Q * CurrentState.R) /
                           CurrentState.I_zz;

            CurrentState.P_dot = P_dot;
            CurrentState.Q_dot = Q_dot;
            CurrentState.R_dot = R_dot;
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

    }
}
