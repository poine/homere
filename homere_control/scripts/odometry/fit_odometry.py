#!/usr/bin/env python

import sys, numpy as np, matplotlib.pyplot as plt
import sklearn.linear_model
# see https://scikit-learn.org/stable/auto_examples/linear_model/plot_ransac.html
import pdb

import julie_misc.plot_utils as jpu
import homere_control.io_dataset as hio

class Regression:

    def __init__(self, ds):
        self.ds = ds
        # interpolate truth at same timestamps as encoders
        self.truth_lvel_body_1 = hio.interpolate(ds.truth_lvel_body, ds.truth_vel_stamp, ds.enc_vel_stamp)
        self.truth_rvel_1 = hio.interpolate(ds.truth_rvel, ds.truth_vel_stamp, ds.enc_vel_stamp)
        self.report_fmt = '''
|         | est      | res std (m)  | res mu (m)| 
|:--------|:--------:|:------------:|:---------:|
| simple: | {:.4f}   | {:.2e}       | {:.2e}    |
| ransac: | {:.4f}   | {:.2e}       | {:.2e}    |
'''
        
    def fit_wheel_radius(self):
        # fit wheel radius using encoder avg rvel and truth vel
        # Y = H.w   Y=truth_vel H=0.5*enc_vel_sum w=wheel_radius 
        print('\n## fitting wheel radius')

        # simple least square (pseudoinvert)
        Y = self.truth_lvel_body_1[:,0]
        H = 0.5*self.ds.enc_vel_sum[:,np.newaxis]
        self.wheel_radius_simple = np.dot(np.linalg.pinv(H), Y)
        self.res_radius_simple = Y - np.dot(H, self.wheel_radius_simple)
        self.res_radius_simple_sigma, self.res_radius_simple_mu = np.std(self.res_radius_simple), np.mean(self.res_radius_simple) 

        # ransac (outlier remover) least square
        self.ransac_wr = sklearn.linear_model.RANSACRegressor(base_estimator=sklearn.linear_model.LinearRegression(fit_intercept=False))
        self.ransac_wr.fit(H, Y)
        self.wheel_radius_ransac = self.ransac_wr.estimator_.coef_
        self.res_radius_ransac = Y - np.dot(H, self.wheel_radius_ransac)
        self.res_radius_ransac_sigma =  np.std(self.res_radius_ransac[self.ransac_wr.inlier_mask_])
        self.res_radius_ransac_mu =  np.mean(self.res_radius_ransac[self.ransac_wr.inlier_mask_])

        print('wheel radius:\n' + self.report_fit_wheel_radius())
        self.wheel_radius = self.wheel_radius_ransac

    def report_fit_wheel_radius(self):
        return self.report_fmt.format(self.wheel_radius_simple[0],
                                      self.res_radius_simple_sigma, self.res_radius_simple_mu,
                                      self.wheel_radius_ransac[0],
                                      self.res_radius_ransac_sigma, self.res_radius_ransac_mu )
        


        
    def fit_wheel_sep(self):
        # fit wheel_sep using encoder diff rvel and truth rvel
        # psi_d = enc_diff * wr / ws
        print('\n## fitting wheel sep')

        # simple least square (pseudoinvert)
        Y = self.truth_rvel_1[:,2]
        H = (-self.ds.enc_vel_dif)[:,np.newaxis]
        wr_ov_ws = np.dot(np.linalg.pinv(H), Y)
        self.res_wheel_sep_simple = Y - np.dot(H, wr_ov_ws)
        self.res_wheel_sep_simple_sigma, self.res_wheel_sep_simple_mu = np.std(self.res_wheel_sep_simple), np.mean(self.res_wheel_sep_simple)
        self.wheel_sep_simple = self.wheel_radius/wr_ov_ws
 
        # ransac (outlier removal) least square
        self.ransac_ws = sklearn.linear_model.RANSACRegressor(base_estimator=sklearn.linear_model.LinearRegression(fit_intercept=False))
        self.ransac_ws.fit(H, Y)
        self.wheel_sep_ransac = self.wheel_radius / self.ransac_ws.estimator_.coef_
        self.res_wheel_sep_ransac = Y - np.dot(H, self.ransac_ws.estimator_.coef_)
        self.res_wheel_sep_ransac_sigma =  np.std(self.res_wheel_sep_ransac[self.ransac_ws.inlier_mask_])
        self.res_wheel_sep_ransac_mu =  np.mean(self.res_wheel_sep_ransac[self.ransac_ws.inlier_mask_])
        
        print('wheel sep:\n'+self.report_fit_wheel_sep())

        self.wheel_sep = self.wheel_sep_ransac


    def report_fit_wheel_sep(self):
        return self.report_fmt.format(self.wheel_sep_simple[0],
                                      self.res_wheel_sep_simple_sigma, self.res_wheel_sep_simple_mu,
                                      self.wheel_sep_ransac[0],
                                      self.res_wheel_sep_ransac_sigma, self.res_wheel_sep_ransac_mu)
        

    
    def plot_residuals(self, info, filename=None):
        figsize=(20.48, 10.24)
        margins = 0.05, 0.07, 0.97, 0.95, 0.14, 0.39
        fig = jpu.prepare_fig(window_title='Odometry parameters regression ({})'.format(info), figsize=figsize, margins=margins)

        inlier_mask = self.ransac_wr.inlier_mask_
        outlier_mask = np.logical_not(inlier_mask)
        ax = plt.subplot(4,1,1)
        #plt.scatter(self.ds.enc_vel_stamp, self.res_radius_simple, marker='.')
        plt.scatter(self.ds.enc_vel_stamp[outlier_mask], self.res_radius_ransac[outlier_mask], color='gold', marker='.', label='Outliers')
        plt.scatter(self.ds.enc_vel_stamp[inlier_mask], self.res_radius_ransac[inlier_mask], color='yellowgreen', marker='.', label='Inliers')
        jpu. decorate(ax, title='wheel radius residuals', xlab='time in s', ylab='m', legend=True, xlim=None, ylim=None)

        _fmt_legend = '$\mu: {:.2e} \quad \sigma: {:.2e}$'
        ax = plt.subplot(4,2,3)
        plt.hist(self.res_radius_simple, normed=True, bins=30)
        jpu. decorate(ax, title='simple', xlab='m', legend=[_fmt_legend.format(self.res_radius_simple_mu, self.res_radius_simple_sigma)])
 
        ax = plt.subplot(4,2,4)
        plt.hist(self.res_radius_ransac[self.ransac_wr.inlier_mask_], normed=True, bins=30)
        jpu. decorate(ax, title='ransac', xlab='m', legend=[_fmt_legend.format(self.res_radius_ransac_mu, self.res_radius_ransac_sigma)])
 
        inlier_mask = self.ransac_ws.inlier_mask_
        outlier_mask = np.logical_not(inlier_mask)
        ax = plt.subplot(4,1,3)
        #plt.scatter(self.ds.enc_vel_stamp, self.res_wheel_sep_simple, marker='.')
        plt.scatter(self.ds.enc_vel_stamp[outlier_mask], self.res_wheel_sep_ransac[outlier_mask], color='gold', marker='.', label='Outliers')
        plt.scatter(self.ds.enc_vel_stamp[inlier_mask], self.res_wheel_sep_ransac[inlier_mask], color='yellowgreen', marker='.', label='Inliers')
        jpu. decorate(ax, title='wheel separation residuals', xlab='time in s', ylab='', legend=True, xlim=None, ylim=None)
        
        ax = plt.subplot(4,2,7)
        plt.hist(self.res_wheel_sep_simple, normed=True, bins=30)
        jpu. decorate(ax, title='simple', legend=[_fmt_legend.format(self.res_wheel_sep_simple_mu, self.res_wheel_sep_simple_sigma)])

        ax = plt.subplot(4,2,8)
        plt.hist(self.res_wheel_sep_ransac[self.ransac_ws.inlier_mask_], normed=True, bins=30)
        jpu. decorate(ax, title='ransac', legend=[_fmt_legend.format(self.res_wheel_sep_ransac_mu, self.res_wheel_sep_ransac_sigma)])
        jpu.savefig(filename)

    def plot_wheel_radius(self, filename=None):
        fig = jpu.prepare_fig(window_title='Wheel radius regression')
        inlier_mask = self.ransac_wr.inlier_mask_
        outlier_mask = np.logical_not(inlier_mask)
        #plt.scatter(self.ds.enc_vel_sum, self.truth_lvel_body_1[:,0], color='gold', marker='.', label='Outliers')
        plt.scatter(self.ds.enc_vel_sum[outlier_mask], self.truth_lvel_body_1[:,0][outlier_mask], color='gold', marker='.', label='Outliers')
        plt.scatter(self.ds.enc_vel_sum[inlier_mask], self.truth_lvel_body_1[:,0][inlier_mask], color='yellowgreen', marker='.', label='Inliers')
        test_enc_vel_sum = np.linspace(np.min(self.ds.enc_vel_sum), np.max(self.ds.enc_vel_sum), 100)[:,np.newaxis]
        test_vel_simple = 0.5 * test_enc_vel_sum * self.wheel_radius_simple
        test_vel_ransac = 0.5 * self.ransac_wr.predict(test_enc_vel_sum)
        plt.plot(test_enc_vel_sum, test_vel_simple, label='simple')
        plt.plot(test_enc_vel_sum, test_vel_ransac, label='ransac')
        jpu.decorate(plt.gca(), title='wheel radius fit', xlab='enc_avg (rad/s)', ylab="v (m/s)", legend=True)#, xlim=[-1, 20], ylim=[-0.1, 1.])
        jpu.savefig(filename)

        
    def plot_wheel_sep(self, filename=None):
        fig = jpu.prepare_fig(window_title='Wheel sep regression')
        inlier_mask = self.ransac_ws.inlier_mask_
        outlier_mask = np.logical_not(inlier_mask)
        plt.scatter(self.ds.enc_vel_dif[outlier_mask], self.truth_rvel_1[:,2][outlier_mask], color='gold', marker='.', label='Outliers')
        plt.scatter(self.ds.enc_vel_dif[inlier_mask], self.truth_rvel_1[:,2][inlier_mask], color='yellowgreen', marker='.', label='Inliers')
        test_enc_vel_dif = np.linspace(np.min(self.ds.enc_vel_dif), np.max(self.ds.enc_vel_dif), 100)[:,np.newaxis]
        test_rvel_simple = test_enc_vel_dif  * -self.wheel_radius / self.wheel_sep_simple
        #test_rvel_ransac = self.ransac_wr.predict(test_enc_vel_dif)
        test_rvel_ransac =  test_enc_vel_dif * -self.wheel_radius / self.wheel_sep_ransac
        plt.plot(test_enc_vel_dif, test_rvel_simple, label='simple')
        plt.plot(test_enc_vel_dif, test_rvel_ransac, label='ransac')
        jpu.decorate(plt.gca(), title='wheel separation fit', xlab='enc_diff (rad/s)', ylab="rvel (rad/s)", legend=True, xlim=[-5., 5.], ylim=[-2.5, 2.5])
        jpu.savefig(filename)
    
        
        
if __name__ == '__main__':
    #_ds = hio.DataSet('/home/poine/work/homere/homere_control/data/odom_gazebo_1.npz', _type='homere')
    #_ds = hio.DataSet('/home/poine/work/oscar.git/oscar/oscar_control/scripts/odometry/odom_data_4.npz', _type='oscar')
    filename, _type = '/mnt/mint18/home/poine/work/julie/julie/julie_control/scripts/julie_odom_data_1.npz', 'homere'
    #filename, _type = '/home/poine/work/homere/homere_control/data/odometry/julie/gazebo_2.npz', 'homere'
    #filename, _type = '/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_3.npz', 'homere'
    #filename, _type = '/home/poine/work/homere/homere_control/data/rosmip/gazebo/rosmip_io_02.npz', 'rosmip'
    #filename, _type = '/mnt/mint18/home/poine/work/homere/homere_control/data/rosmip/gazebo/rosmip_2_io_09_sine_2.npz', 'rosmip'
    type = 'oscar'
    filename = '/mnt/mint18/home/poine/work/julie/julie/julie_control/scripts/julie_odom_data_1.npz' if len(sys.argv) < 2 else sys.argv[1]
    _ds = hio.DataSet(filename, type)
    reg = Regression(_ds)
    #hio.plot_interp(_ds.truth_lvel_body, _ds.truth_stamp, reg.truth_lvel_body_1, _ds.enc_vel_stamp)
    reg.fit_wheel_radius()
    reg.fit_wheel_sep()
    reg.plot_residuals(info=filename)
    reg.plot_wheel_radius()
    reg.plot_wheel_sep()
    plt.show()
    
