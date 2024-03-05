from plyfile import PlyData, PlyElement
import numpy as np

data = [
0.794339, 0.476194, -0.377181,
0.61059, 0.505641, 0.609514,
0.700887, 0.663248, 0.262412,
-0.23602, 0.858743, 0.454813,
0.577742, 0.444866, 0.684331,
0.572664, 0.0499688, 0.818266,
0.751459, 0.216431, 0.623271,
0.861936, 0.234124, 0.449724,
0.581275, 0.380534, 0.719245,
0.474261, 0.589044, 0.654297,
0.425744, 0.560549, 0.710301,
0.745568, 0.163773, 0.645993,
0.779963, 0.295904, 0.55145,
0.27158, 0.951732, 0.143002,
-0.143317, 0.914278, 0.378889,
0.706227, 0.650852, 0.278632,
0.731934, 0.596416, 0.329486,
0.0809674, 0.797445, 0.597934,
0.753959, -0.29232, 0.588298,
0.547847, 0.68189, 0.484654,
0.710657, 0.127017, 0.691978,
0.409963, 0.384253, 0.827212,
0.664869, 0.46596, 0.583808,
0.763292, 0.592772, 0.256917,
0.690204, 0.602185, 0.401237,
0.316649, 0.704672, 0.634958,
0.314591, 0.240955, 0.918136,
0.323815, 0.599599, 0.731864,
0.666447, 0.703947, -0.245576,
0.459103, 0.704095, 0.541733,
0.564893, 0.82381, -0.0472513,
0.657072, 0.716444, 0.234444,
0.525787, 0.712172, 0.465143,
0.561326, 0.725842, 0.397577,
0.234163, 0.457376, 0.85789,
0.878229, 0.292382, 0.378453,
0.359203, 0.800129, 0.480381,
0.723485, 0.490899, 0.485374,
0.872759, -0.0376162, 0.486699,
0.681462, 0.574747, 0.453074,
0.923899, 0.303008, 0.233657,
0.561254, 0.797998, 0.219528,
0.521972, 0.720498, -0.45654,
-0.0892955, 0.166845, 0.981931,
0.715561, 0.278031, 0.640837,
0.943106, 0.229736, 0.240359,
0.971198, -0.189444, 0.144517,
0.616994, 0.657983, 0.431714,
0.602509, -0.051753, 0.796433,
0.046442, 0.401174, 0.914824,
0.75555, 0.341615, 0.558967,
0.380539, 0.455473, 0.80482,
0.379725, 0.602791, 0.701749,
0.477533, 0.452143, 0.753345,
0.364478, 0.0495346, 0.929893,
0.34095, 0.90613, 0.250363,
0.299574, 0.824767, 0.479598,
0.0695599, 0.994366, -0.0799802,
0.537019, 0.619951, 0.572077,
0.703651, 0.612659, 0.359895,
0.823577, -0.171845, 0.540545,
0.203076, 0.250563, 0.946561,
0.447003, 0.849853, -0.279174,
0.597854, 0.78633, 0.155739,
0.953275, 0.297929, -0.0500427,
0.484791, 0.851063, 0.201666,
0.870034, 0.484711, -0.0899809,
0.126481, 0.986861, 0.100534,
0.470935, 0.807706, 0.354728,
0.832538, 0.198476, 0.517192,
0.822992, 0.563822, 0.0692065,
0.63659, 0.670785, 0.380526,
0.926164, 0.28178, 0.250638,
0.63951, 0.570007, 0.515867,
0.813673, 0.352971, 0.461895,
0.366552, 0.919707, -0.140636,
0.22269, -0.218048, 0.950192,
-0.402214, 0.914821, -0.0364122,
0.237133, 0.318007, 0.917954,
0.176153, 0.0518098, 0.982998,
0.650919, 0.256775, 0.714402,
0.740655, 0.200443, 0.64129,
0.528965, 0.780252, 0.333769,
0.661154, 0.284946, 0.694033,
0.314357, 0.549067, 0.774406,
0.239564, 0.843203, 0.481267,
0.625466, 0.655295, 0.423534,
0.779847, 0.0660498, 0.622476,
0.724548, 0.313459, 0.613818,
0.66042, -0.199532, 0.7239,
0.802044, 0.583163, -0.129017,
0.403691, 0.581726, 0.706137,
0.704893, 0.34245, 0.621171,
0.926397, 0.375861, -0.0227579,
0.709483, 0.704651, -0.0100174,
0.547923, 0.815947, 0.184418,
0.596813, 0.394423, 0.698745,
0.296126, 0.903778, 0.309021,
-0.0819251, 0.575341, 0.813801,
0.267271, 0.72228, 0.63787,
0.0669036, 0.528191, 0.846486,
0.422522, 0.89461, 0.145423,
0.852542, 0.498452, -0.157218,
0.795717, 0.42921, 0.427333,
0.825557, 0.517723, 0.224543,
0.821572, 0.239491, 0.517363,
0.0999673, 0.821042, 0.562047,
0.870312, 0.490674, -0.0423846,
0.837402, 0.488815, 0.244579,
-0.0155066, 0.23325, 0.972293,
0.139273, 0.194793, 0.970906,
0.82091, 0.39972, 0.407836,
0.207151, 0.739189, 0.640849,
0.307167, 0.440927, 0.843346,
0.305112, 0.951013, 0.0498124,
-0.11302, 0.0648206, 0.991476,
-0.00923896, 0.860692, 0.509043,
0.314978, 0.693222, 0.648253,
0.396631, 0.483961, 0.780042,
0.694, -0.214204, 0.687372,
0.670839, 0.360515, 0.648077,
0.59366, 0.410416, 0.69219,
0.223617, 0.404006, 0.887003,
0.754181, -0.116228, 0.646299,
0.726086, -0.142693, 0.672635,
0.700453, 0.488047, 0.520745,
0.37524, 0.383519, 0.843865,
0.618232, 0.6901, 0.376233,
0.12601, 0.905278, 0.4057,
0.750304, 0.0900675, 0.654929,
0.663519, 0.278992, 0.694195,
0.16948, 0.528719, 0.831705,
0.214286, 0.725951, 0.653512,
0.0828437, 0.849897, 0.520396,
0.2654, 0.948887, 0.170814,
0.346806, 0.647696, 0.678392,
0.927971, -0.0546046, 0.36863,
0.276501, 0.923551, 0.265709,
0.247538, 0.625597, 0.739833,
0.310129, 0.763486, 0.566488,
0.395869, 0.430943, 0.81091,
0.652223, 0.721342, 0.232961,
0.306411, 0.922404, 0.235123,
0.612351, 0.605671, 0.508123,
0.591184, 0.796089, -0.129399,
0.171937, 0.691438, 0.701677,
0.105909, 0.498768, 0.86024,
0.72574, -0.0695435, 0.684445,
0.241682, 0.750853, 0.614662,
0.652399, -0.0449565, 0.756541,
0.662151, 0.747426, 0.0539568,
0.539809, 0.516932, 0.66437,
0.469216, 0.405546, 0.784454,
0.559961, 0.827455, 0.0419712,
0.6637, 0.737709, -0.12364,
0.643242, 0.633804, 0.429572,
0.813578, 0.269477, 0.51524,
0.27705, 0.873403, 0.400513,
0.782287, 0.455173, 0.425258,
0.614801, 0.673355, 0.410624,
0.835185, 0.378258, 0.399232,
0.727159, 0.518165, 0.450271,
0.51223, 0.849973, 0.123156,
0.0396859, -0.012388, 0.999135,
0.724194, 0.631944, 0.276025,
0.635278, 0.704611, 0.31614,
0.773667, 0.296888, 0.55973,
0.459264, 0.492444, 0.739307,
0.556796, 0.805681, 0.202129,
0.679007, 0.420538, 0.601745,
0.74868, 0.448078, 0.488574,
0.556601, 0.567173, 0.607051,
0.613255, 0.789559, -0.022683,
0.540434, -0.371721, 0.754821,
0.830764, 0.34095, 0.439983,
0.175797, 0.963566, 0.201583,
0.417664, -0.120835, 0.900531,
0.395426, 0.480062, 0.783057,
0.475789, 0.737163, 0.479809,
0.0295293, 0.947963, 0.317007,
0.580151, 0.300052, 0.757227,
-0.0764925, 0.754714, 0.651579,
0.451512, 0.705524, 0.546235,
0.849298, 0.165065, 0.501444,
0.313155, 0.156409, 0.936734,
-0.0792083, 0.989238, 0.123021,
0.53201, 0.481189, 0.696723,
0.835606, 0.17472, 0.520803,
0.684767, 0.446029, 0.576327,
0.756396, 0.546143, 0.35999,
0.262927, 0.505857, 0.821571,
0.911422, 0.139439, 0.387126,
0.266587, 0.425067, 0.865014,
0.250748, 0.853165, 0.457421,
0.516769, 0.807288, 0.285019,
0.944543, 0.319805, 0.0745806,
0.58681, 0.809031, -0.0335092,
0.636231, 0.681787, 0.361078,
0.823114, 0.411486, 0.39136,
0.844837, 0.528787, 0.0814591,
0.0989353, 0.259755, 0.960593,
0.530007, 0.72425, 0.441083,
0.516562, 0.434323, 0.737921,
0.7247, 0.59103, 0.35425,
0.0431719, 0.971442, 0.233315,
0.416087, 0.449346, 0.790544,
0.309675, 0.548882, 0.776421,
0.490414, 0.787155, 0.374007,
0.545119, 0.276332, 0.791508,
-0.238756, 0.968494, -0.0708151,
0.425942, 0.288694, 0.857455,
0.392951, 0.519771, 0.758569,
0.23698, 0.340191, 0.910006,
0.65225, 0.117101, 0.748904,
0.837753, 0.363296, 0.40766,
0.841822, 0.409271, 0.351899,
0.31706, 0.497074, 0.807707,
0.750084, -0.0872711, 0.655559,
0.299972, 0.406622, 0.862946,
0.62301, 0.702774, 0.343464,
0.147038, 0.893693, 0.423902,
0.772482, 0.577118, 0.264965,
0.341004, 0.539015, 0.770181,
0.256499, 0.42401, 0.868576,
0.436078, 0.773591, 0.459774,
0.749204, 0.405349, 0.523819,
0.744036, 0.603448, -0.286813,
0.780857, 0.0943653, 0.617541,
0.359105, 0.806622, 0.469473,
0.333312, 0.319683, 0.886964,
0.847577, 0.370246, 0.380173,
0.442793, 0.527765, 0.724844,
0.577462, 0.0443337, 0.815213,
0.510042, -0.166434, 0.843894,
0.333531, 0.805209, 0.490302,
0.751408, 0.302437, 0.586445,
0.0373569, 0.933196, 0.357422,
0.627946, 0.555948, 0.544614,
0.139099, 0.9783, 0.15356,
0.47422, 0.79239, 0.383711,
0.472099, 0.720698, 0.507659,
0.522369, 0.741538, 0.421014,
0.666683, 0.731697, 0.141961,
0.794815, 0.58562, 0.159119,
-0.00435036, 0.371692, 0.928346,
0.49929, 0.34836, 0.793319,
0.75664, 0.630617, 0.172679,
0.47052, 0.847584, 0.245381,
0.596048, 0.443639, 0.669262,
0.495097, 0.413335, 0.76422,
-0.00666365, 0.961021, 0.276394,
0.327143, -0.261118, 0.908182,
0.606939, 0.12831, 0.784323,
0.414125, 0.460069, 0.785389,
0.376931, 0.721376, 0.580981,
0.637208, 0.689215, 0.34489,
0.812491, 0.239077, 0.531696,
0.107089, 0.706165, 0.699902,
0.287801, 0.945052, -0.155073,
0.596805, 0.656152, 0.461831,
-0.00855103, 0.234253, 0.972138,
0.506586, 0.839828, 0.195086,
0.992034, -0.125964, -0.0014326,
0.331402, 0.501668, 0.799063,
0.310406, 0.886508, 0.343151,
0.404936, 0.287558, 0.86795,
0.742459, 0.305735, 0.596055,
0.284769, 0.735136, 0.615209,
0.648501, 0.760709, -0.0277266,
0.948251, 0.294491, 0.11872,
0.143992, 0.560942, 0.815236,
0.862829, 0.431703, 0.262981,
0.940394, 0.303571, 0.153308,
0.830307, 0.524315, -0.188904,
0.690847, 0.603416, 0.39827,
0.646362, 0.737574, -0.195451,
0.635339, 0.754802, 0.16315,
0.393586, 0.854044, 0.340144,
0.204393, 0.882435, 0.423712,
0.829067, -0.0408778, 0.557653,
0.296146, 0.798419, 0.524236,
0.462866, 0.0297339, 0.88593,
0.505514, 0.494535, 0.707029,
0.582256, 0.619653, 0.526316,
0.373477, 0.71868, 0.586527,
0.797923, 0.314215, 0.514381,
0.747671, 0.0403605, 0.662841,
-0.00519803, 0.999969, 0.0059416,
0.681078, 0.534539, 0.500401,
0.470605, 0.56857, 0.674729,
0.614454, 0.302947, 0.72847,
0.387082, 0.881256, 0.271212,
0.341434, 0.706542, 0.619856,
0.203856, 0.859454, 0.468809,
0.695148, 0.54616, 0.467417,
0.638336, 0.676298, 0.367625,
0.559304, 0.348965, 0.751933,
0.725267, 0.289346, 0.624713,
0.696002, 0.713573, -0.0799747,
0.481355, 0.626792, 0.612723,
-0.0481319, 0.238005, 0.970071,
0.38518, 0.347412, 0.854951,
0.231641, 0.342918, 0.910357,
0.149372, 0.718043, 0.679781,
0.827569, 0.55943, 0.04655,
0.528327, 0.705987, 0.47165,
0.481637, 0.647254, 0.590837,
0.326907, -0.121989, 0.93715,
0.468826, 0.654564, 0.593083,
0.974547, 0.0462119, 0.219367,
0.477928, 0.122826, 0.869769,
0.196086, 0.098466, 0.97563,
0.491271, 0.820555, 0.292135,
0.587399, 0.809225, 0.0107859,
0.889954, 0.307016, 0.33723,
0.867588, 0.322194, 0.37879,
0.534692, 0.355036, 0.766846,
0.228158, 0.705803, 0.670661,
0.637964, 0.714831, 0.286389,
0.266296, 0.742098, 0.615123,
0.279345, 0.37334, 0.884638,
0.732684, 0.486597, 0.475813,
0.104758, 0.340743, 0.934302,
0.127364, 0.987811, 0.0894915,
0.713837, 0.700225, 0.0110379,
0.693482, 0.218708, 0.686476,
0.486348, 0.870012, -0.0809043,
0.761966, 0.0856427, 0.64193,
0.340816, 0.705677, 0.62118,
0.113562, -0.29383, 0.949088,
0.212707, 0.544537, 0.811317,
0.881131, 0.264274, 0.392132,
0.764323, 0.125574, 0.632488,
0.0548231, 0.707059, 0.705026,
0.741744, 0.575911, 0.343718,
0.847873, 0.465148, 0.254458,
0.574499, 0.710608, 0.406187,
0.666099, 0.274589, 0.693479,
0.728031, 0.396336, 0.559364,
0.524896, 0.256072, 0.811733,
0.196177, 0.591835, 0.781822,
0.68932, 0.643558, 0.332673,
0.510822, 0.274133, 0.814808,
0.514614, 0.256483, 0.818162,
0.211875, 0.614156, 0.760211,
0.709324, 0.523074, 0.472498,
0.229567, 0.449783, 0.863131,
0.792306, 0.0845092, 0.604242,
0.832741, 0.513088, -0.208046,
-0.182913, 0.957774, 0.221836,
0.543103, 0.535358, 0.646862,
0.747563, 0.642075, 0.169967,
0.562694, 0.48004, 0.673006,
0.875302, 0.419287, 0.240923,
0.804152, 0.565347, 0.183636,
0.730882, 0.296539, 0.614716,
-0.0235826, 0.606458, 0.794765,
0.968333, 0.113484, 0.222379,
0.432023, 0.809377, 0.397825,
0.371884, 0.515212, 0.772178,
0.534608, 0.446602, 0.717454,
0.7876, 0.184232, 0.588001,
0.334231, 0.905332, 0.262036,
0.828291, 0.473005, 0.300332,
0.0871567, 0.991271, 0.0989219,
0.477295, 0.70259, 0.527784,
0.430476, 0.791629, 0.433606,
0.729212, 0.665942, 0.157386,
0.358648, 0.506209, 0.784298,
0.550373, 0.366788, 0.750037,
0.892331, 0.423076, 0.157325,
0.523097, 0.629445, 0.574602,
0.385959, 0.162273, 0.908132,
0.166185, 0.913748, 0.370739,
0.649881, 0.320399, 0.689202,
0.0910665, 0.525639, 0.845819,
0.654457, 0.135836, 0.743797,
0.178028, 0.31182, 0.933314,
0.434673, 0.25212, 0.864578,
0.632971, 0.764037, 0.124877,
0.696885, 0.534028, 0.478712,
0.765526, 0.392383, 0.509907,
0.90231, 0.281643, 0.326365,
0.348223, 0.664286, 0.661411,
0.678168, 0.646792, 0.348926,
0.535274, 0.529224, 0.658334,
0.843263, 0.497619, 0.203181,
0.814901, 0.286896, 0.503614,
0.463143, 0.862544, 0.203753,
0.496419, 0.415101, 0.762404,
0.753696, 0.176373, 0.633116,
0.35967, 0.848388, 0.388428,
0.874145, 0.482351, -0.0566491,
0.240558, 0.612293, 0.753146,
-0.142254, 0.809941, 0.568999,
0.484199, 0.794544, 0.366402,
0.770905, 0.318681, 0.551497,
0.450893, 0.882852, -0.131407,
0.362561, 0.891299, 0.27228,
0.281711, 0.411019, 0.867007,
0.0676773, 0.983984, 0.164908,
0.814407, 0.56202, 0.144478,
0.433516, 0.901142, -0.00270417,
0.558577, 0.266291, 0.785545,
0.545308, 0.194961, 0.815248,
0.849859, 0.419848, -0.31854,
0.555206, 0.759297, 0.339431,
0.393431, 0.355272, 0.847935,
-0.0658393, 0.952731, 0.296594,
0.483845, 0.234967, 0.843021,
0.277439, 0.896213, 0.346164,
0.374645, 0.866254, 0.330522,
-0.167179, 0.68868, 0.705528,
0.983712, 0.164599, -0.0722293,
-0.0218673, 0.556949, 0.830259,
0.546386, 0.826605, 0.134859,
0.544332, 0.603073, 0.5831,
0.413149, 0.726241, 0.549438,
0.131698, 0.481197, 0.866663,
0.356566, 0.782441, 0.510536,
0.666313, 0.601529, 0.44067,
0.080866, 0.526223, 0.846493,
0.385616, 0.852802, 0.352177,
0.55078, 0.413528, 0.725008,
0.40514, 0.871493, 0.276336,
0.970246, 0.169997, 0.172404,
0.552277, 0.696136, 0.458677,
0.637791, 0.34268, 0.689778,
0.624423, 0.0749311, 0.777484,
0.185441, 0.9134, 0.362371,
0.941439, 0.121949, 0.314358,
0.221201, 0.666319, 0.712102,
0.575842, 0.211412, 0.789754,
-0.221699, 0.896746, 0.383011,
0.310746, 0.572998, 0.75836,
0.103585, 0.508012, 0.855099,
0.884412, 0.163256, 0.437222,
0.859621, 0.502321, 0.0934105,
0.664122, 0.319023, 0.676141,
0.661885, 0.40514, 0.63069,
0.593223, 0.257976, 0.762584,
0.292667, 0.889541, 0.350803,
0.425554, 0.21568, 0.878855,
0.704179, 0.557524, 0.439658,
0.187247, 0.725504, 0.662255,
0.689463, 0.432703, 0.580869,
0.778079, 0.479552, 0.405738,
0.685222, 0.65463, 0.319265,
0.413272, 0.66458, 0.622527,
0.420155, 0.780926, 0.462195,
0.648872, 0.667225, 0.365753,
0.743202, 0.341171, 0.575546,
0.443262, 0.468402, 0.764277,
0.596035, 0.496977, 0.630679,
0.429656, 0.507181, 0.747103,
0.343618, 0.904073, 0.254124,
0.393149, 0.321283, 0.861517,
0.641101, 0.719007, 0.268361,
0.644293, 0.401594, 0.650852,
0.534274, 0.404459, 0.74227,
0.728805, 0.625576, 0.278385,
0.487533, 0.385714, 0.783285,
0.995131, -0.0369781, 0.0913637,
0.288481, 0.153975, 0.945024,
0.92054, 0.288823, 0.263035,
0.50319, 0.138617, 0.852986,
0.640376, 0.735381, 0.22166,
0.671342, 0.486417, 0.559194,
0.387175, 0.423935, 0.818764,
0.36119, 0.856094, 0.369654,
-0.225303, 0.354326, 0.907575,
0.742634, 0.625991, 0.237972,
0.270599, 0.378642, 0.885102,
0.142617, 0.709936, 0.689674,
0.0555424, 0.7622, 0.644954,
0.663202, 0.637778, 0.391667,
0.694049, 0.612561, 0.378239,
0.29552, 0.639765, 0.709485,
0.539723, 0.473611, 0.695982,
0.977525, 0.210783, 0.00399789,
0.867133, 0.372903, 0.330188,
0.563632, 0.230065, 0.79334,
0.641276, -0.0327128, 0.766613,
0.224816, 0.974388, -0.00507167,
0.553821, 0.7798, 0.291882,
0.409073, 0.215918, 0.886588,
0.226406, 0.351429, 0.908426,
0.369008, 0.520918, 0.769726,
0.677794, 0.406866, 0.612418,
0.626953, 0.516768, 0.582993,
0.53884, 0.717875, 0.440802,
0.824534, 0.33045, 0.45929,
-0.0207005, 0.914205, 0.404724,
0.650959, 0.729137, 0.211214,
0.44971, 0.543306, 0.708929,
0.821456, 0.0936229, 0.562534,
0.79956, 0.597352, 0.0622424,
0.576618, 0.457519, 0.676896,
0.368443, 0.493373, 0.78793,
0.582293, 0.805518, -0.109885,
0.691013, 0.412516, 0.593575,
0.382024, 0.10628, 0.918021,
0.963197, 0.25333, -0.0898648,
0.202621, 0.635664, 0.7449,
0.516049, 0.439819, 0.735019,
0.909428, 0.366176, 0.197117,
0.839289, 0.375321, 0.393354,
0.710368, 0.702831, -0.0375081,
0.730307, 0.0893626, 0.677249,
0.585456, 0.43482, 0.684231,
0.412742, 0.748679, 0.51877,
0.664538, 0.541224, 0.515234
]


def save_ply(path: str):
    xyz = np.array(data).reshape(-1, 3)
    norm = np.zeros_like(xyz)
    print(xyz.shape)

    dtype_full = [(attribute, 'f4') for attribute in ['x', 'y', 'z', 'nx', 'ny', 'nz']]

    elements = np.empty(xyz.shape[0], dtype=dtype_full)
    attributes = np.concatenate((xyz, norm), axis=1)
    elements[:] = list(map(tuple, attributes))
    el = PlyElement.describe(elements, 'vertex')
    PlyData([el]).write(path)


save_ply("test.ply")