# AR523 Assignment 2: Force-Controlled Writing & Haptic Material Identification

Completed solution for [AR523-Assignment2-ForceControl](https://github.com/lakshay-ctrl/AR523-Assignment2-ForceControl).

| Path | Contents |
|---|---|
| `main.py` | Completed code (all 8 TODOs + bonus cursive writing) |
| `utils.py` | Given infrastructure, **unmodified** |
| `outputs/` | All generated figures and JSON results (default word "Robot", pattern "circle") |
| `report/Report.pdf` | Written report (also `Report.html`; source in `report/src/`) |
| `partA/PartA_Handwritten.pdf` | Handwritten Part A derivations |

## Run

```bash
pip install -r requirements.txt
python main.py --part all            # everything; add --live for the real-time dashboard
python main.py --part 1              # shapes
python main.py --part 2              # impedance writing
python main.py --part 3              # admittance writing
python main.py --part 4              # comparison
python main.py --part 5              # material identification
python main.py --part bonus          # cursive "robot" + spiral/infinity/heart
```

## Results

| | Impedance | Admittance |
|---|---|---|
| Force RMSE, "Robot" + circle | 0.53 N | 0.51 N |
| Force RMSE, cursive "robot" (bonus) | 0.47 N | 0.40 N |

Material identification: K̂ = 179 / 314 / 385 N/m → soft / medium / hard (3/3 correct).

To rebuild the report: `npm install katex@0.16.11 playwright`, then `node report/src/build_report.js`.
