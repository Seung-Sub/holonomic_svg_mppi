import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_best_particle(df_path="/tmp/svgd_best_particle_log.csv"):
    # """
    # CSV 컬럼: [Iteration, Row(=timestep), Col(=제어입력 index), Value]
    # 각 Iteration마다 (각 Col) 전체 time step(Row)에 대한 평균을 구해 플롯.
    # """
    df = pd.read_csv(df_path)

    # Iteration, Col 기준으로 groupby, Row(=time step) 평균
    gb = df.groupby(["Iteration", "Col"], as_index=False)["Value"].mean()
    # => Iteration, Col, Value(=time-step 평균)

    # pivot: index=Iteration, columns=Col, values=Value
    pivot_best = gb.pivot(index="Iteration", columns="Col", values="Value")

    # 예) Col=0->Vx, 1->Vy, 2->Wz (가정)
    pivot_best.columns = ["Vx", "Vy", "Wz"]

    plt.figure(figsize=(8,5))
    pivot_best.plot(marker='o')
    plt.title("Average Best Particle Value per Iteration (Averaged over TimeSteps)")
    plt.xlabel("Iteration")
    plt.ylabel("Best Particle (Mean over time steps)")
    plt.grid(True)
    plt.legend(title="Dimension")
    # plt.savefig("svgd_best_particle_mean_over_timestep.png")
    plt.show()

def plot_adaptive_cov(df_path="/tmp/svgd_adaptive_cov_log.csv"):
    # """
    # CSV 컬럼: [Iteration, TimeStep, Dim, CovValue]
    # 모든 TimeStep에 대한 평균(각 Dim별)을 Iteration마다 구해 플롯.
    # """
    df_cov = pd.read_csv(df_path)

    # Iteration, Dim 기준으로 groupby, TimeStep 평균
    gb_cov = df_cov.groupby(["Iteration", "Dim"], as_index=False)["CovValue"].mean()
    # => Iteration, Dim, CovValue(=time-step 평균)

    # pivot: index=Iteration, columns=Dim, values=CovValue
    pivot_cov = gb_cov.pivot(index="Iteration", columns="Dim", values="CovValue")

    # 예) Dim=0->Vx_cov, 1->Vy_cov, 2->Wz_cov
    pivot_cov.columns = ["Vx_cov", "Vy_cov", "Wz_cov"]

    plt.figure(figsize=(8,5))
    pivot_cov.plot(marker='o')
    plt.title("Adaptive Covariance (Averaged over TimeSteps)")
    plt.xlabel("Iteration")
    plt.ylabel("CovValue (Mean over TimeSteps)")
    plt.grid(True)
    plt.legend(title="Dimension")
    # plt.savefig("svgd_adaptive_cov_mean_over_timestep.png")ㄴ
    plt.show()

if __name__ == "__main__":
    plot_best_particle()
    plot_adaptive_cov()
