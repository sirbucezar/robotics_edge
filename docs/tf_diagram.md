# Transform tree

The robot publishes a single connected transform (TF) tree. This diagram was
captured with `ros2 run tf2_tools view_frames.py` while the full stack was
running, and redrawn for legibility. Rates are measured, not nominal.

<svg viewBox="0 0 820 470" xmlns="http://www.w3.org/2000/svg"
     font-family="Calibri, Helvetica, Arial, sans-serif">
  <defs>
    <marker id="b" markerWidth="9" markerHeight="7" refX="9" refY="3.5"
            orient="auto"><path d="M0,0 L9,3.5 L0,7 z" fill="#444"/></marker>
  </defs>
  <style>
    .f  { fill:#ffffff; stroke:#222; stroke-width:1.6; rx:5; }
    .fs { fill:#f4f4f4; stroke:#666; stroke-width:1.3; rx:5; }
    .nm { font-size:15px; font-weight:700; fill:#111; }
    .sb { font-size:11px; fill:#555; }
    .ed { stroke:#444; stroke-width:1.4; fill:none; marker-end:url(#b); }
    /* White stroke painted under the fill, so an edge passing behind a label
       is knocked out rather than striking through it. */
    .el { font-size:11px; fill:#333; stroke:#ffffff; stroke-width:3.5px;
          paint-order:stroke; }
    .er { font-size:10px; fill:#777; stroke:#ffffff; stroke-width:3.5px;
          paint-order:stroke; }
    .sb { stroke:#ffffff; stroke-width:3px; paint-order:stroke; }
  </style>

  <rect class="f" x="300" y="20" width="210" height="52"/>
  <text class="nm" x="405" y="43" text-anchor="middle">map</text>
  <text class="sb" x="405" y="60" text-anchor="middle">world frame, fixed to the saved map</text>

  <path class="ed" d="M405,72 L405,132"/>
  <text class="el" x="415" y="96">amcl</text>
  <text class="er" x="415" y="112">10.2 Hz · localization correction</text>

  <rect class="f" x="300" y="134" width="210" height="52"/>
  <text class="nm" x="405" y="157" text-anchor="middle">odom</text>
  <text class="sb" x="405" y="174" text-anchor="middle">continuous, drifts over time</text>

  <path class="ed" d="M405,186 L405,246"/>
  <text class="el" x="415" y="210">limo_base</text>
  <text class="er" x="415" y="226">50.2 Hz · wheel and inertial odometry</text>

  <rect class="f" x="300" y="248" width="210" height="52"/>
  <text class="nm" x="405" y="271" text-anchor="middle">base_link</text>
  <text class="sb" x="405" y="288" text-anchor="middle">robot body origin</text>

  <path class="ed" d="M340,300 C240,330 150,340 130,368"/>
  <path class="ed" d="M405,300 L405,368"/>
  <path class="ed" d="M470,300 C570,330 660,340 680,368"/>
  <text class="er" x="292" y="322" text-anchor="end">static</text>
  <text class="er" x="416" y="330">static</text>
  <text class="er" x="520" y="322">static</text>

  <rect class="fs" x="30" y="370" width="200" height="62"/>
  <text class="nm" x="130" y="393" text-anchor="middle">laser_link</text>
  <text class="sb" x="130" y="410" text-anchor="middle">YDLIDAR, 220 degree arc</text>
  <text class="sb" x="130" y="425" text-anchor="middle">10 Hz scan</text>

  <rect class="fs" x="305" y="370" width="200" height="62"/>
  <text class="nm" x="405" y="393" text-anchor="middle">camera_link</text>
  <text class="sb" x="405" y="410" text-anchor="middle">Dabai DC1 colour, 71 deg HFOV</text>
  <text class="sb" x="405" y="425" text-anchor="middle">approx. 18 cm above the floor</text>

  <rect class="fs" x="580" y="370" width="200" height="62"/>
  <text class="nm" x="680" y="393" text-anchor="middle">imu_link</text>
  <text class="sb" x="680" y="410" text-anchor="middle">chassis IMU</text>
  <text class="sb" x="680" y="425" text-anchor="middle">100 Hz</text>
</svg>

## Frames

| Frame | Parent | Broadcaster | Rate | Purpose |
|---|---|---|---|---|
| `map` | — | — | — | World frame, fixed to the saved classroom map |
| `odom` | `map` | `amcl` | 10.2 Hz | Localization correction. Only one node may publish this edge. |
| `base_link` | `odom` | `limo_base` | 50.2 Hz | Wheel and inertial odometry from the chassis controller |
| `laser_link` | `base_link` | static | — | Lidar mount |
| `camera_link` | `base_link` | static | — | Camera mount, about 18 cm above the floor |
| `imu_link` | `base_link` | static | — | Inertial measurement unit mount |

## The single-publisher rule

During mapping, Cartographer publishes `map` to `odom`. During the mission,
AMCL publishes it. Never run both.

Two publishers on one transform do not fall back to each other. The tree
alternates between them, and every pose in the stack becomes unreliable in a way
that resembles random navigation failure. The launch file `nav2_amcl.launch.py`
documents this requirement at the top of the file.

## Why camera height matters to the tree

The `base_link` to `camera_link` transform places the camera about 18 cm above
the floor. At that height a ray to a distant foot runs nearly parallel to the
ground, so a few pixels of bounding-box jitter change a floor-intersection range
estimate by metres.

Measured at 4 m with 5 px of noise: ground-plane range error is 1.4 m mean and
5.2 m at the 95th percentile, against 0.30 m and 0.80 m for the shoulder-width
estimator. The tracker therefore trusts floor intersection only within 2.5 m.
