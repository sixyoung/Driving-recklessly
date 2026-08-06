import matplotlib.pyplot as plt

def load_kappa(filename):
    idx, raw, smooth = [], [], []
    with open(filename) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) >= 3:
                try:
                    idx.append(int(parts[0]))
                    raw.append(float(parts[1]))
                    smooth.append(float(parts[2]))
                except ValueError:
                    continue
    return idx, raw, smooth


def plot_kappa(filename):
    idx, raw, smooth = load_kappa(filename)

    plt.figure(figsize=(10, 6))
    plt.plot(idx, raw, "r--", label="Raw Kappa", linewidth=1.5)
    plt.plot(idx, smooth, "b-", label="Smoothed Kappa", linewidth=2)

    plt.xlabel("Path Point Index")
    plt.ylabel("Curvature κ [1/m]")
    plt.title("Raw vs Smoothed Curvature")
    plt.legend()
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    file = "/home/bob/文档/备份/demo05/src/driver/planner/src/test/test/txt/kappa.txt"
    plot_kappa(file)
