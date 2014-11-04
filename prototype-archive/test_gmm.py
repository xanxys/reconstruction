from __future__ import print_function
import itertools
import numpy as np
from scipy import linalg
import pylab as pl
import matplotlib as mpl
from sklearn import mixture
import os.path


def generate_two_gaussian(n_samples=500):
    """
    Sample 2D 2-GMM
    n_samples: Number of samples per component
    return: [n_samples * 2, 2]
    """
    np.random.seed(0)
    C = np.array([[0., -0.1], [1.7, .4]])
    X = np.r_[np.dot(np.random.randn(n_samples, 2), C),
              .7 * np.random.randn(n_samples, 2) + np.array([-6, 3])]
    return X

def load_samples(dir_path):
    x = open(os.path.join(dir_path, "debug_points_interior_2d.ply"))

    points = []
    for (i, line) in enumerate(x):
        if i < 12:
            continue
        x, y, z, r, g, b = map(float, line.split())
        points.append([x, y, z, r, g, b])
    return np.array(points)

X = load_samples("scan-20140801-18:54-gakusei-small-1")
print(X.shape)

# Fit a mixture of gaussians with EM using five components
gmm = mixture.GMM(n_components=10, covariance_type='full')
gmm.fit(X)

# Fit a dirichlet process mixture of gaussians using five components
dpgmm = mixture.DPGMM(n_components=20, covariance_type='full', alpha=0.1)
dpgmm.fit(X)

color_iter = itertools.cycle(['r', 'g', 'b', 'c', 'm'])

for i, (clf, title) in enumerate([(gmm, 'GMM')]):
                                  #(dpgmm, 'Dirichlet Process GMM')]):
    splot = pl.subplot(1, 1, 1 + i)
    Y_ = clf.predict(X)
    for i, (mean, covar, color) in enumerate(zip(
            clf.means_, clf._get_covars(), color_iter)):
        v, w = linalg.eigh(covar)
        u = w[0] / linalg.norm(w[0])
        # as the DP will not use every component it has access to
        # unless it needs it, we shouldn't plot the redundant
        # components.
        if not np.any(Y_ == i):
            continue
        pl.scatter(X[Y_ == i, 0], X[Y_ == i, 1], .8, color=color)

        # Plot an ellipse to show the Gaussian component
        angle = np.arctan(u[1] / u[0])
        angle = 180 * angle / np.pi  # convert to degrees
        ell = mpl.patches.Ellipse(mean, v[0], v[1], 180 + angle, color=color)
        ell.set_clip_box(splot.bbox)
        ell.set_alpha(0.5)
        splot.add_artist(ell)

    pl.xlim(-10, 10)
    pl.ylim(-3, 6)
    pl.xticks(())
    pl.yticks(())
    pl.title(title)

pl.show()