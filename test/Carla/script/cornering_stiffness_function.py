import numpy as np
import pandas as pd

# ------------------------------------------------------------
# USER PARAMETERS (797F loaded case)
# ------------------------------------------------------------
lf = 4.80          # m
lr = 2.40          # m
m = 623690.0       # kg
Iz = 7.19e6        # kg*m^2

CSV_PATH = "vehicle_dynamics.csv"

# Filtering thresholds
VX_MIN = 2.0                 # m/s, avoid low-speed division issues
MAX_ABS_STEER_DEG = 8.0      # keep near-linear region
MAX_ABS_SLIP_DEG = 5.0       # keep near-linear region

def deg(x):
    return np.degrees(x)

def main():
    df = pd.read_csv(CSV_PATH)

    # Required columns
    t = df["t"].to_numpy()
    vx = df["vx_body"].to_numpy()
    vy = df["vy_body"].to_numpy()
    r  = df["yaw_rad"].to_numpy()
    delta = df["steer"].to_numpy()

    # Numerical derivatives
    vy_dot = np.gradient(vy, t)
    r_dot  = np.gradient(r, t)

    # Standard small-angle bicycle-model slip-angle approximation
    eps = 1e-6
    vx_safe = np.where(np.abs(vx) < eps, eps, vx)

    alpha_f = delta - (vy + lf * r) / vx_safe
    alpha_r = -(vy - lr * r) / vx_safe

    # Linear-region mask
    mask = (
        (np.abs(vx) > VX_MIN) &
        (np.abs(deg(delta)) < MAX_ABS_STEER_DEG) &
        (np.abs(deg(alpha_f)) < MAX_ABS_SLIP_DEG) &
        (np.abs(deg(alpha_r)) < MAX_ABS_SLIP_DEG)
    )

    # Left-hand side
    y1 = m * (vy_dot + vx * r)
    y2 = Iz * r_dot

    # Build least-squares system
    # y1 = Cf * alpha_f + Cr * alpha_r
    # y2 = lf*Cf * alpha_f - lr*Cr * alpha_r
    A_top = np.column_stack([alpha_f, alpha_r])
    A_bot = np.column_stack([lf * alpha_f, -lr * alpha_r])
    A = np.vstack([A_top, A_bot])

    b = np.hstack([y1, y2])

    # Apply mask to both stacked parts
    idx = np.where(mask)[0]
    A_used = np.vstack([A_top[idx], A_bot[idx]])
    b_used = np.hstack([y1[idx], y2[idx]])

    # Least squares
    theta, residuals, rank, s = np.linalg.lstsq(A_used, b_used, rcond=None)
    Cf_est, Cr_est = theta

    print("Estimated axle-equivalent cornering stiffness:")
    print(f"Cf = {Cf_est:.6e} N/rad")
    print(f"Cr = {Cr_est:.6e} N/rad")
    print()
    print(f"Samples used: {len(idx)} / {len(df)}")
    print(f"Rank: {rank}")
    if len(residuals) > 0:
        print(f"Residual sum of squares: {residuals[0]:.6e}")

if __name__ == "__main__":
    main()