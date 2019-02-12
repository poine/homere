#!/usr/bin/env python
#-*- coding: utf-8 -*-
import matplotlib.pyplot as plt

import homere_control.io_dataset as odm_ds
import fit_odometry as odm_ft

all_examples = [['homere_gazebo',
                 [['/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_1.npz', 'homere', 'homere_gazebo_1'],
                  ['/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_2.npz', 'homere', 'homere_gazebo_2'],
                  ['/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_3.npz', 'homere', 'homere_gazebo_3'],
                  ['/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_4.npz', 'homere', 'homere_gazebo_4'],
                  ['/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_5.npz', 'homere', 'homere_gazebo_5'],
                  ['/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_6.npz', 'homere', 'homere_gazebo_6'],
                  ['/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_7.npz', 'homere', 'homere_gazebo_7'],
                  ['/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_8.npz', 'homere', 'homere_gazebo_8'],
                  ['/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_9.npz', 'homere', 'homere_gazebo_9'],
                  ['/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_10.npz', 'homere', 'homere_gazebo_10'],
                  ['/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_11_random_lrvel.npz', 'homere', 'homere_gazebo_11'],
                  ['/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_12_step_pwm_sum.npz', 'homere', 'homere_gazebo_12'],
                  ['/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_13.npz', 'homere', 'homere_gazebo_13'],
                  ['/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_13_random_pwm_sum.npz', 'homere', 'homere_gazebo_13_1'],
                  ['/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_14_random_pwm_dif.npz', 'homere', 'homere_gazebo_14'],
                  ['/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_16_step_pwm_10_sum.npz', 'homere', 'homere_gazebo_16_1'],
                  ['/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_16_step_pwm_20_sum.npz', 'homere', 'homere_gazebo_16_2'],
                  ['/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_16_step_pwm_30_sum.npz', 'homere', 'homere_gazebo_16_3'],
                  ['/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_16_step_pwm_40_sum.npz', 'homere', 'homere_gazebo_16_4'],
                  ['/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_16_step_pwm_sum.npz', 'homere', 'homere_gazebo_16_5'],
                  ['/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_17_sine_pwm_15_sum.npz', 'homere', 'homere_gazebo_17_1'],
                  ['/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_17_sine_pwm_30_sum.npz', 'homere', 'homere_gazebo_17_2'],
                  ['/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_17_sine_pwm_40_sum.npz', 'homere', 'homere_gazebo_17_3'],
                  ['/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_17_sine_pwm_sum.npz', 'homere', 'homere_gazebo_17_5']]],
                ['oscar_smocap',
                 [['/home/poine/work/oscar.git/oscar/oscar_control/scripts/odometry/odom_data_1.npz', 'oscar', 'oscar_smocap_1'],
                  ['/home/poine/work/oscar.git/oscar/oscar_control/scripts/odometry/odom_data_2.npz', 'oscar', 'oscar_smocap_2'],
                  ['/home/poine/work/oscar.git/oscar/oscar_control/scripts/odometry/odom_data_3.npz', 'oscar', 'oscar_smocap_3'],
                  ['/home/poine/work/oscar.git/oscar/oscar_control/scripts/odometry/odom_data_4.npz', 'oscar', 'oscar_smocap_4']]],
                ['julie_gazebo',
                 [['/home/poine/work/julie/julie/julie_control/scripts/julie_odom_data_1.npz', 'julie', 'julie_gazebo_1'],
                  ['/home/poine/work/homere/homere_control/data/odometry/julie/gazebo_2.npz', 'julie', 'julie_gazebo_2'],
                  ['/home/poine/work/homere/homere_control/data/odometry/julie/gazebo_3.npz', 'julie', 'julie_gazebo_3'],
                  ['/home/poine/work/homere/homere_control/data/odometry/julie/gazebo_4.npz', 'julie', 'julie_gazebo_4']]]]

plot_dir = '/home/poine/work/homere/doc/web/plots/'
md_dir   = '/home/poine/work/homere/doc/web/'


def write_md_page(filename, title, content, layout='default'):
    print('writing markdown to {}'.format(filename))
    with open(filename, 'w') as f:
        f.write('''---
title: {}
layout: {}
---
'''.format(title, layout))
        f.write(content)

def write_html_include(filename, content):
    print('writing html to {}'.format(filename))
    with open(filename, 'w') as f:
        f.write(content)

#
# Summary 
#

all_summarry_plots = [('truth_2d', odm_ds.plot2d),
                      ('truth_wheels', odm_ds.plot_encoders),
                      ('truth_vel', odm_ds.plot_truth_vel),
                      ('enc_stats', odm_ds. plot_encoders_stats),
                      ('enc_3D', odm_ds.plot_encoders_3D)]

def write_img_link(img_src):
    return '''<a href="{}" target="_blank"><img src="{}" border="0" align="center" width="250"></img></a>\n'''.format(img_src, img_src)
        
def gen_summary_mardown(examples):
    html_filename = md_dir + '_includes/odometry_examples.html'
    content = ''
    for _section, _section_examples in examples:
        content += '\n## {}\n<table style="border-collapse: collapse; border-spacing: 20px; text-align: center; border: 0px solid black;">\n'.format(_section)
        #content += '<tr><td colspan="4">{}</td></tr>\n'.format(_section)
        for filename, _type , _name in _section_examples:
            content += '''
<tr>
  <td style="text-align: left; padding: 15px;" valign="top" width="50">
	{}

<a id="{}"></a>
  </td>
'''.format(_name, _name)
            for _p_name, _f in all_summarry_plots:
                content += '<td valign="top">{}</td>'.format(write_img_link('../plots/{}_{}.png'.format(_name, _p_name)))
            content+= "</tr>\n"
        content += '\n</table>\n'
    write_html_include(html_filename, content)

def gen_summary_plots(examples):
    for _section, _section_examples in examples:
        for filename, _type , _name in _section_examples:
            _ds = odm_ds.DataSet(filename, _type)
            for _p_name, _f in all_summarry_plots:
                _f(_ds, filename=plot_dir+_name+'_{}.png'.format(_p_name))
            plt.close('all')
        
def make_summary(examples, make_plots):
    gen_summary_mardown(examples)
    if make_plots: gen_summary_plots(examples)



#
# Regression1
#

all_reg1_plots = [
    ('reg_dd1_wr',  odm_ft.Regression.plot_wheel_radius, odm_ft.Regression.report_fit_wheel_radius),
    ('reg_dd1_ws',  odm_ft.Regression.plot_wheel_sep, odm_ft.Regression.report_fit_wheel_sep),
    ('reg_dd1_res', odm_ft.Regression.plot_residuals, None)]

def make_regression_dif_drive(examples, make_plots=False):
    html_filename = md_dir + '_includes/regression1.html'
    content = ''
    for _section, _section_examples in examples:
        content += '''

## {}

<table style="border-collapse: collapse; border-spacing: 20px; text-align: center; border: 0px solid black;">
<tr><th>Exp</th><th>Reg Radius</th><th>Reg Separation</th><th>Residuals</th></tr>\n\n'''.format(_section)
        for filename, _type , _name in _section_examples:
            _ds = odm_ds.DataSet(filename, _type)
            reg = odm_ft.Regression(_ds)
            reg.fit_wheel_radius()
            reg.fit_wheel_sep()
            if make_plots:
                reg.plot_residuals(info=_name, filename=plot_dir+_name+'_reg_dd1_res.png')
                reg.plot_wheel_radius(filename=plot_dir+_name+'_reg_dd1_wr.png')
                reg.plot_wheel_sep(filename=plot_dir+_name+'_reg_dd1_ws.png')
                plt.close('all')
            comment = 'size: {} enc'.format(len(_ds.enc_stamp))
            _ds_2d_img = '../plots/{}_truth_2d.png'.format(_name)
            content += '''<tr>
  <td style="text-align: left; padding: 15px;" valign="top" width="50">
	<div class="hover_img">
          <a href="../examples#{}">{} <span><img src="{}" alt="image" height="480" /></span> </a> 
        </div>

        {}
  </td>
'''.format(_name, _name, _ds_2d_img, comment)
            for _p_name, _fp, _fr in all_reg1_plots:
                img_with_link = write_img_link('../plots/{}_{}.png'.format(_name, _p_name))
                desc = _fr(reg) if _fr is not None else ''
                content += '  <td valign="top">\n    {}\n    <br>\n\n<div markdown="1"  style="float: right">\n\n{}\n\n</div>\n\n  </td>\n'.format(img_with_link, desc)
            content+= "</tr>\n"
        content += '\n</table>\n'
    write_html_include(html_filename, content)
     
if __name__ == '__main__':
    #make_summary(all_examples, make_plots=False)
    make_regression_dif_drive(all_examples, make_plots=False)
