# キャリブレーションモデルの数式

`backend/model` の各キャリブレーションモデルが計算している数式をまとめる。
入出力の単位は、位置が mm、角度が deg。たわみの計算は内部で m・kg・N・rad を使う。

---

## 1. 共通の記号と関数

### 1.1 回転行列（ZYX オイラー角）

姿勢角 $\boldsymbol e = [e_1, e_2, e_3]$（コード上の `[roll, pitch, yaw]`）から回転行列を作る。

$$
R(\boldsymbol e) = R_z(e_1)\,R_y(e_2)\,R_x(e_3)
$$

逆変換は次のとおり。

$$
e_2 = \operatorname{atan2}\!\left(-R_{31},\ \sqrt{R_{11}^2 + R_{21}^2}\right),\quad
e_1 = \operatorname{atan2}(R_{21}, R_{11}),\quad
e_3 = \operatorname{atan2}(R_{32}, R_{33})
$$

$e_2 = \pm 90^\circ$ の特異姿勢では $e_1$ と $e_3$ を分けられないので、$e_1 = 0$、$e_3 = \operatorname{atan2}(\operatorname{sign}(e_2)\,R_{12},\ R_{22})$ とする。

### 1.2 工具オフセット

手先（フランジ）姿勢 $(\boldsymbol p, \boldsymbol e)$ に工具 $k$ のオフセット $\boldsymbol t_k \in \mathbb R^3$ を加え、工具先端の位置を求める。

$$
\boldsymbol p^{\mathrm{tool}} = \boldsymbol p + R(\boldsymbol e)\,\boldsymbol t_k
$$

### 1.3 決定係数

$N$ 点・$D$ 次元の観測 $y_{nd}$ と予測 $\hat y_{nd}$ に対し、平均 $\bar y_d$ は列（次元）ごとにとる。

$$
R^2 = 1 - \frac{\sum_{n,d} (y_{nd} - \hat y_{nd})^2}{\sum_{n,d} (y_{nd} - \bar y_d)^2}
$$

---

## 2. 順運動学

関節角 $\boldsymbol q = [q_1, \dots, q_6]$ [deg] から手先姿勢を求める。
J4 と J6 は回転の向きが逆なので、符号 $\boldsymbol s = [1, 1, 1, -1, 1, -1]$ を掛けて使う。
7 番目のリンクは関節を持たない（フランジまで）。

### 2.1 理想キネマ（`mode = "ideal"`）

修正 DH パラメータ $(a_i, \alpha_i, d_i, \theta^0_i)$（$i = 1..7$）を使う。

$$
T_i = R_x(\alpha_i)\,\mathrm{Tr}_x(a_i)\,R_z(\theta_i)\,\mathrm{Tr}_z(d_i),\qquad
\theta_i = \theta^0_i + s_i q_i\ \ (i \le 6),\quad \theta_7 = \theta^0_7
$$

$$
T = T_1 T_2 \cdots T_7,\qquad \boldsymbol p = T_{1:3,4},\qquad \boldsymbol e = R^{-1}(T_{1:3,1:3})
$$

公称値はリンク長 $l_1..l_6$ から次のように決まる。

| $i$ | $a$ | $\alpha$ | $d$ | $\theta^0$ |
|---|---|---|---|---|
| 1 | 0 | 0 | $l_1$ | 90 |
| 2 | $l_2$ | 90 | 0 | 90 |
| 3 | $l_3$ | 0 | 0 | 0 |
| 4 | $l_4$ | 90 | $l_5$ | 0 |
| 5 | 0 | −90 | 0 | 0 |
| 6 | 0 | 90 | 0 | 0 |
| 7 | 0 | 0 | $l_6$ | 0 |

推定するのは原点角度 $\theta^0_1..\theta^0_6$。`*_tool` モードでは、データに現れる工具のオフセット $\boldsymbol t_k$ も推定する。

### 2.2 たわみ補正キネマ（`mode = "actual"`）

各リンクを幾何パラメータ $\boldsymbol g_i = [x_i, y_i, z_i, u_i, v_i, w_i]$（$i = 1..7$）で表す。

$$
T_i = \mathrm{Tr}(x_i + \Delta x_i,\ y_i + \Delta y_i,\ z_i)\;
R_x(u_i)\;R_y(v_i + \Delta v_i)\;R_z(w_i + s_i q_i + \Delta w_i)
$$

$$
T = T_1\,R_y(\psi_1)\,T_2\,T_3 \cdots T_7
$$

$\Delta$ の各項と $\psi_1$ は、重力によるたわみ（2.3 節）を表す。

| 対象 | たわみ量 |
|---|---|
| $\Delta w_1, \Delta w_2, \Delta w_4, \Delta w_5, \Delta w_6$ | 関節のねじれ $\delta_1, \delta_2, \delta_4, \delta_5, \delta_6$ |
| $\Delta w_3$ | $\delta_3 + \phi_2$（J3 のねじれ + 第 2 アーム先端のたわみ角） |
| $\Delta y_3$ | $w_2$（第 2 アーム先端のたわみ量 [mm]） |
| $\Delta x_4$ | $w_4$（第 3 アーム先端のたわみ量 [mm]） |
| $\Delta v_4$ | $\phi_4$（第 3 アーム先端のたわみ角） |
| $\psi_1$ | ベースの倒れ（第 1 リンクの後に挟む $y$ 軸まわりの回転） |

公称値 $\boldsymbol g^{\mathrm{ideal}}$ は、リンク長 $l_1..l_6$ とオフセット $l_{1a}, l_{5a}, l_{6a}$ から決まる。

| $i$ | $x$ | $y$ | $z$ | $u$ | $v$ | $w$ |
|---|---|---|---|---|---|---|
| 1 | 0 | 0 | $l_{1a}$ | 0 | 0 | 90 |
| 2 | $l_2$ | 0 | $l_1 - l_{1a}$ | 90 | 0 | 90 |
| 3 | $l_3$ | 0 | 0 | 0 | 0 | 0 |
| 4 | $l_4$ | $-l_{5a}$ | 0 | 90 | 0 | 0 |
| 5 | 0 | 0 | $l_5 - l_{5a}$ | −90 | 0 | 0 |
| 6 | 0 | $-l_{6a}$ | 0 | 90 | 0 | 0 |
| 7 | 0 | 0 | $l_6 - l_{6a}$ | 0 | 0 | 0 |

保存する幾何誤差は $\boldsymbol g - \boldsymbol g^{\mathrm{ideal}}$。

#### 推定するパラメータ（`calibration_mode`）

各モードは、前のモードの推定対象を含む。

| モード | 追加で推定するパラメータ |
|---|---|
| `none` | なし（キネマを固定） |
| `origin_actual` | 第 1 リンクの $x, y, z, u, v, w$（ベースの位置・姿勢）、関節原点 $w_2..w_6$ |
| `arm_actual` | $x_2,\ x_3,\ x_4,\ y_4,\ y_6$ |
| `kinema_actual` | $y_2,\ x_5,\ y_5,\ x_6,\ u_2,\ u_3,\ v_3,\ u_4,\ u_5,\ u_6,\ x_7$ |
| `all_actual` | 剛性率 $\kappa_2..\kappa_5$（J2〜J5 の関節剛性） |

### 2.3 重力たわみ

#### 記号

| 記号 | 意味 |
|---|---|
| $W_k,\ M^x_k,\ M^y_k,\ M^z_k$ | リンク $k$（1〜6）の質量と 1 次モーメント |
| $m_L,\ \boldsymbol r_L = (r_x, r_y, r_z)$ | 可搬物の質量と重心 [m] |
| $\boldsymbol g = G\,\hat{\boldsymbol g}$ | 重力ベクトル（$G = 9.80665$、$\hat{\boldsymbol g}$ は重力方向。既定は $[0, 0, -1]$） |
| $k_1..k_6,\ EI_1, EI_2,\ k_b$ | 関節剛性、アームの曲げ剛性、ベース剛性（機種の値） |
| $\kappa_1..\kappa_9$ | 剛性率（初期値 1）。$\kappa_1..\kappa_6$ が関節、$\kappa_7, \kappa_8$ がアーム、$\kappa_9$ がベース |

$\theta_j = q_j$ [rad] とし、$c_j = \cos\theta_j$、$s_j = \sin\theta_j$、$c_{23} = \cos(\theta_2 + \theta_3)$、$s_{23} = \sin(\theta_2 + \theta_3)$ と書く。

#### 質量をまとめる

$$
m_1 = W_1,\quad m_2 = W_2,\quad m_3 = W_3 + W_4,\quad m_5 = W_5 + W_6
$$

$$
\begin{aligned}
l_{g1} &= (l_1 - l_{1a}) + M^z_1/W_1, & l_{g2} &= M^x_1/W_1, & l_{g3} &= M^x_2/W_2, & l_{g4} &= -M^y_2/W_2,\\
l_{g5} &= \frac{M^x_3 + M^x_4}{m_3}, & l_{g6} &= \frac{-M^y_3 + M^z_4 + l_5 W_4}{m_3}, & l_{g7} &= \frac{-M^y_5 + M^z_6 + l_6 W_6}{m_5}
\end{aligned}
$$

#### 重力モーメント

重力を J1 と一緒に回る座標系で表す。

$$
\boldsymbol g' = R_z(\theta_1)^{\top} \boldsymbol g
$$

手首の向きは次の回転で表す。

$$
R_5 = R_x(\theta_2 + \theta_3)\,R_y(-\theta_4)\,R_x(\theta_5)
$$

手先側から順に、各関節まわりのモーメント（1 次モーメントのベクトル × 重力）を積み上げる。

$$
\begin{aligned}
\boldsymbol a_5 &= [0,\ m_5 l_{g7},\ 0] + m_L\,[\,r_y c_6 - r_x s_6,\ l_6 + r_z,\ r_y s_6 + r_x c_6\,], &
\boldsymbol M_5 &= (R_5 \boldsymbol a_5) \times \boldsymbol g'\\
\boldsymbol a_3 &= [0,\ m_3 l_{g6} + m_5 l_5,\ m_3 l_{g5} + m_5 l_4] + m_L\,[0,\ l_5,\ l_4], &
\boldsymbol M_3 &= \boldsymbol M_5 + (R_x(\theta_2 + \theta_3)\,\boldsymbol a_3) \times \boldsymbol g'\\
\boldsymbol a_2 &= [0,\ m_2 l_{g4},\ m_2 l_{g3} + (m_3 + m_5) l_3] + m_L\,[0,\ 0,\ l_3], &
\boldsymbol M_2 &= \boldsymbol M_3 + (R_x(\theta_2)\,\boldsymbol a_2) \times \boldsymbol g'\\
\boldsymbol a_1 &= [0,\ m_1 l_{g2} + m_{2\text{-}5}\,l_2,\ m_1 l_{g1} + m_{2\text{-}5}\,(l_1 - l_{1a})] + m_L\,[0,\ l_2,\ l_1 - l_{1a}], &
\boldsymbol M_1 &= \boldsymbol M_2 + \boldsymbol a_1 \times \boldsymbol g'
\end{aligned}
$$

ここで $m_{2\text{-}5} = m_2 + m_3 + m_5$。

#### 関節のねじれ

各関節軸まわりのモーメント $\tau_j$ を剛性で割り、ねじれ角を求める。

$$
\tau_1 = M_{1,z},\quad \tau_2 = M_{2,x},\quad \tau_3 = M_{3,x},\quad
\tau_4 = M_{5,y}\,c_{23} + M_{5,z}\,s_{23},\quad
\tau_5 = \boldsymbol M_5 \cdot R_5 \boldsymbol e_x,\quad
\tau_6 = \boldsymbol M_5 \cdot R_5 \boldsymbol e_y
$$

$$
\delta_j = \frac{\tau_j}{10^4\,k_j}\,\kappa_j \qquad (j = 1..6)
$$

#### アームのたわみ（片持ち梁）

第 2 アーム（長さ $l_3$）と第 3 アーム（長さ $l_5$）を、先端にモーメント $M$ と荷重 $F$ を受ける片持ち梁として扱う。

$$
M_{a2} = M_{3,x},\quad F_2 = (m_3 + m_5 + m_L)(g'_y c_2 + g'_z s_2),\quad EI'_2 = 10^4 EI_1 / \kappa_7
$$

$$
M_{a3} = M_{5,x},\quad F_3 = (m_5 + m_L)(g'_y s_{23} - g'_z c_{23}),\quad EI'_3 = 10^4 EI_2 / \kappa_8
$$

$$
\phi_2 = \frac{2 M_{a2} l_3 - F_2 l_3^2}{2 EI'_2},\qquad
w_2 = 1000 \cdot \frac{3 M_{a2} l_3^2 - 2 F_2 l_3^3}{6 EI'_2}
$$

$$
\phi_4 = \frac{2 M_{a3} l_5 - F_3 l_5^2}{2 EI'_3},\qquad
w_4 = 1000 \cdot \frac{3 M_{a3} l_5^2 - 2 F_3 l_5^3}{6 EI'_3}
$$

#### ベースの倒れ

$$
\psi_1 = -\frac{M_{1,x}}{10^4\,k_b}\,\kappa_9
$$

$\kappa$ はたわみやすさ（コンプライアンス）の倍率として効く。保存する剛性値は（機種の剛性値）$/\kappa$。

---

## 3. 関節の角度伝達誤差

減速機の回転に同期する周期誤差を、指令角 $q_j$ に加えて実角度 $\tilde q_j$ とする。周期 $P_{jm}$ [deg] は機種ごとに固定。

$$
\tilde q_j = q_j + \sum_m \left( a_{jm} \sin\frac{2\pi q_j}{P_{jm}} + b_{jm} \cos\frac{2\pi q_j}{P_{jm}} \right)
$$

最適化では、線形な係数 $a_{jm}, b_{jm}$ を推定する。こうすると、位相の周回や振幅の符号で解が割れない。
保存・表示のときだけ、振幅と位相に直す。

$$
A_{jm} \sin\!\left(\frac{2\pi q_j}{P_{jm}} + \varphi_{jm}\right),\qquad
A_{jm} = \sqrt{a_{jm}^2 + b_{jm}^2},\quad \varphi_{jm} = \operatorname{atan2}(b_{jm}, a_{jm})
$$

---

## 4. 観測モデル

工具先端の位置 $\boldsymbol p_n$（ロボット座標）を、計測器で観測される値 $\hat{\boldsymbol y}_n = h(\boldsymbol p_n)$ に変換する。
パラメータを持つ観測モデルは、ロボットのパラメータと同時に推定する。

| 種類 | 観測値 $h(\boldsymbol p_n)$ | 推定するパラメータ |
|---|---|---|
| `identity` | $\boldsymbol p_n$ | なし |
| `relative` | $\boldsymbol p_n - \boldsymbol p_{n_0(s)}$（$n_0(s)$ は系列 $s$ で時刻が最も早い点） | なし |
| `rigid` | $R(\boldsymbol e_M)\,\boldsymbol p_n + \boldsymbol t_M$ | $\boldsymbol t_M, \boldsymbol e_M$（6 個） |
| `scalar` | $\gamma\,(\hat{\boldsymbol a} \cdot \boldsymbol p_n) + \beta$（$\hat{\boldsymbol a}$ は単位ベクトルに正規化した計測方向） | $\gamma, \beta$ |

### rigid の初期値（Kabsch 法）

対応する点の組 $(\boldsymbol p_n, \boldsymbol y_n)$ から、最小二乗で最もよく重なる剛体変換を求める。

$$
H = \sum_n (\boldsymbol p_n - \bar{\boldsymbol p})(\boldsymbol y_n - \bar{\boldsymbol y})^{\top} = U \Sigma V^{\top}
$$

$$
R_M = V\,\operatorname{diag}\!\left(1,\ 1,\ \operatorname{sign}\det(V U^{\top})\right) U^{\top},\qquad
\boldsymbol t_M = \bar{\boldsymbol y} - R_M \bar{\boldsymbol p}
$$

$\operatorname{diag}$ の第 3 成分で、鏡映の解を避ける。

---

## 5. キネマキャリブレーション（`kinema`, `kinema_joint`）

### 5.1 予測値

| モデル | 工具先端の位置 $\boldsymbol p_n$ |
|---|---|
| `kinema` | $\mathrm{FK}(\boldsymbol q_n)$ に工具オフセット $\boldsymbol t_{k_n}$ を加える |
| `kinema_joint` | $\mathrm{FK}(\tilde{\boldsymbol q}_n)$ に工具オフセット $\boldsymbol t_{k_n}$ を加える（$\tilde{\boldsymbol q}_n$ は伝達誤差を加えた実角度） |

`kinema_joint` では、FK は常にたわみ補正キネマを使う。予測値は $\hat{\boldsymbol y}_n = h(\boldsymbol p_n)$。

### 5.2 目的関数

ロボット側のパラメータ $\boldsymbol\theta_r$ と観測側のパラメータ $\boldsymbol\theta_o$ を 1 本のベクトルにし、同時に最小二乗推定する。

$$
\min_{\boldsymbol\theta_r,\,\boldsymbol\theta_o}\ \frac12 \sum_n \left\| \boldsymbol y_n - h\!\left(\boldsymbol p_n(\boldsymbol\theta_r);\ \boldsymbol\theta_o\right) \right\|^2
$$

- 解法は `scipy.optimize.least_squares` の既定（Trust Region Reflective 法、数値微分のヤコビアン）。
- 残差の数が未知数の数より少ない場合は、推定しない。

### 5.3 段階的な同定（`kinema_joint` の `pattern`）

各段階で推定対象を切り替えながら、5.2 節の最小二乗を順に解く。

| pattern | 段階（キネマを推定するか, 伝達誤差を推定する関節） |
|---|---|
| `kinema_only` | (する, なし) |
| `trans_j1` | (しない, J1) |
| `trans_all` | (しない, J1〜J6) |
| `kinema_trans_j1` | (する, J1) |
| `kinema_trans_all` | (する, J1〜J6) |
| `kinema_then_trans_j1` | (する, なし) → (しない, J1) |
| `kinema_then_trans_all` | (する, なし) → (しない, J1〜J6) |

「キネマを推定する」段階では、`calibration_mode` のパラメータ（2.2 節）を動かす。「しない」段階では、`none` として固定する。

---

## 6. 軌跡キャリブレーション（`trajectory`）

時系列の関節角軌道（FM）と、計測器で取った手先軌跡（BT）を突き合わせる。

### 6.1 予測値

- 動作 $s$ の関節角軌道: $\{(t^s_k,\ \boldsymbol q^s_k)\}$
- 計測点 $n$: 動作番号 $s_n$、計測時刻 $\tau_n$、計測値 $\boldsymbol y_n$（計測器座標の XYZ）

ロボットの時刻は、計測時刻に動作ごとの時刻ずれ $\Delta_s$ を足したものとする。
関節角は、その時刻で各関節ごとに線形補間する（軌道の範囲外は端の値）。

$$
\boldsymbol q_n = \operatorname{interp}\!\left(\tau_n + \Delta_{s_n};\ t^{s_n}_k,\ \boldsymbol q^{s_n}_k\right)
$$

$$
\boldsymbol p_n = \mathrm{FK}_{\mathrm{actual}}(\tilde{\boldsymbol q}_n) \text{ に工具 1 のオフセット } \boldsymbol t_1 \text{ を加えたもの},\qquad
\hat{\boldsymbol y}_n = R(\boldsymbol e_M)\,\boldsymbol p_n + \boldsymbol t_M
$$

観測モデルは常に `rigid`。

### 6.2 推定するパラメータ

どの段階でも、時刻ずれ $\Delta_s$ と計測器の座標 $(\boldsymbol t_M, \boldsymbol e_M)$ を推定する。そのうえで、5.3 節のパターンに従い、キネマと伝達誤差を段階ごとに加える。
キネマだけを固定して時刻ずれと座標のみを推定する `time_only` パターン（(しない, なし)）もある。

- 計測器の座標変換は、第 1 リンクの $x, y, z, u, v, w$（ベースの位置・姿勢）と区別できない。そのため、ベースのパラメータは推定対象から外す。
- 時刻ずれは $\Delta_s = \Delta^0_s + \varepsilon_s$ と分け、補正量 $\varepsilon_s$ だけを最適化変数にする。絶対時刻が大きい場合でも、数値微分の刻みが大きくなりすぎないようにするため。

### 6.3 初期値

**時刻ずれ $\Delta^0_s$。** 手先の速さは座標系によらないので、ロボット側と計測側の速さの時系列を重ね合わせて求める。

1. 刻み $h = 0.01$ s の等間隔時刻で、2 つの速さの時系列を作る。
   - ロボット側 $v^R_i$: 関節角軌道を補間して手先位置を求め、その速さ $\|d\boldsymbol p / dt\|$ をとる（中心差分）。
   - 計測側 $v^M_i$: 計測値を補間した軌跡から、同じように速さをとる。
2. 重なる区間の平均二乗差が最小になるずらし量 $L$ を探す。重なりが計測側の約半分未満になる $L$ は探索範囲から外す。

$$
L^* = \arg\min_{L} \operatorname{mean}_{i} \left( v^R_{i+L} - v^M_i \right)^2
$$

$$
\Delta^0_s = t^R_0 - \tau^M_0 + L^* h
$$

ここで $t^R_0$ はロボット側、$\tau^M_0$ は計測側の最初の時刻。

**計測器の座標。** $\tau_n + \Delta^0_{s_n}$ が軌道の時間範囲に入る点だけを残し、その点の組 $(\boldsymbol p_n, \boldsymbol y_n)$ から Kabsch 法（4 節）で決める。
以降の最小二乗も、この範囲内の点だけで行う。

---

## 7. 工具キャリブレーション（`tool_calib`）

機構は補正せず、工具オフセットだけを推定する。入力は計測した手先姿勢 $(\boldsymbol p_n, \boldsymbol e_n)$。

$$
\min_{\{\boldsymbol t_k\}}\ \frac12 \sum_n \left\| \boldsymbol y_n - h\!\left(\boldsymbol p_n + R(\boldsymbol e_n)\,\boldsymbol t_{k_n}\right) \right\|^2
$$

観測モデル $h$ の既定は `relative`。

---

## 8. 関節補正（`joint_calib`）

関節角 $q$ [deg] に対する補正量を、周期正弦波の和で近似する。

$$
\hat y(q) = \sum_m A_m \sin\!\left( 2\pi \left( \frac{q}{P_m} + \frac{\varphi_m}{360} \right) \right)
$$

$A_m, \varphi_m$ は `scipy.optimize.curve_fit` で推定する（初期値はすべて 1）。
推定後、$A_m < 0$ の項は $A_m \to -A_m$、$\varphi_m \to \varphi_m + 180$ として、振幅を正に揃える。

---

## 9. ローカル座標キャリブレーション（`local_calib`）

実測座標 $\boldsymbol x_n$ を基準座標 $\boldsymbol y_n$ へ移す剛体変換を推定する。

$$
\hat{\boldsymbol y}_n = \boldsymbol t + R(\boldsymbol e)\,\boldsymbol x_n,\qquad
\min_{\boldsymbol t, \boldsymbol e} \sum_n \|\boldsymbol y_n - \hat{\boldsymbol y}_n\|^2
$$

初期値は 0 で、`scipy.optimize.leastsq` で解く。逆変換は $\boldsymbol x = R(\boldsymbol e)^{\top}(\boldsymbol y - \boldsymbol t)$。

### 3 点からのローカル座標系

原点 $\boldsymbol p_0$、X 軸上の点 $\boldsymbol p_x$、XY 平面上の点 $\boldsymbol p_y$ から、座標系を組み立てる。

$$
\hat{\boldsymbol x} = \frac{\boldsymbol p_x - \boldsymbol p_0}{\|\boldsymbol p_x - \boldsymbol p_0\|},\quad
\hat{\boldsymbol z} = \frac{(\boldsymbol p_x - \boldsymbol p_0) \times (\boldsymbol p_y - \boldsymbol p_0)}{\|\cdot\|},\quad
\hat{\boldsymbol y} = \hat{\boldsymbol z} \times \hat{\boldsymbol x}
$$

$$
[\boldsymbol t, \boldsymbol e] = \left[\boldsymbol p_0,\ R^{-1}([\hat{\boldsymbol x}\ \hat{\boldsymbol y}\ \hat{\boldsymbol z}])\right]
$$
