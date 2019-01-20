#!/usr/bin/env python
#-*- coding: utf-8 -*-

import odom_dataset as odm_ds
import fit_odometry as odm_ft

all_examples = [['homere_gazebo', [['/home/poine/work/homere/homere_control/data/odom_gazebo_1.npz', 'homere', 'homere_gazebo_1'],
                                   ['/home/poine/work/homere/homere_control/data/odom_gazebo_2.npz', 'homere', 'homere_gazebo_2'],
                                   ['/home/poine/work/homere/homere_control/data/homere/gazebo/homere_io_3.npz', 'homere', 'homere_gazebo_3']]],
                ['oscar_smocap', [['/home/poine/work/oscar.git/oscar/oscar_control/scripts/odometry/odom_data_1.npz', 'oscar', 'oscar_smocap_1'],
                                  ['/home/poine/work/oscar.git/oscar/oscar_control/scripts/odometry/odom_data_2.npz', 'oscar', 'oscar_smocap_2'],
                                  ['/home/poine/work/oscar.git/oscar/oscar_control/scripts/odometry/odom_data_3.npz', 'oscar', 'oscar_smocap_3'],
                                  ['/home/poine/work/oscar.git/oscar/oscar_control/scripts/odometry/odom_data_4.npz', 'oscar', 'oscar_smocap_4']]],
                ['julie_gazebo', [['/home/poine/work/julie/julie/julie_control/scripts/julie_odom_data_1.npz', 'julie', 'julie_gazebo_1'],
                                  ['/home/poine/work/homere/homere_control/data/odometry/julie/gazebo_2.npz', 'julie', 'julie_gazebo_2'],
                                  ['/home/poine/work/homere/homere_control/data/odometry/julie/gazebo_3.npz', 'julie', 'julie_gazebo_3'],
                                  ['/home/poine/work/homere/homere_control/data/odometry/julie/gazebo_4.npz', 'julie', 'julie_gazebo_4']]]]

plot_dir = '/home/poine/work/homere/homere_control/doc/web/plots/'
md_dir   = '/home/poine/work/homere/homere_control/doc/web/'


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

        
def gen_summary_mardown(examples):
    #md_filename = md_dir + 'examples.md'
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
  </td>
  <td valign="top"><img src="../plots/{}_truth_2d.png" border="0" align="center" width="250"></img></td>
  <td valign="top"><img src="../plots/{}_truth_wheels.png" border="0" align="center" width="250"></img></td>
  <td valign="top"><img src="../plots/{}_truth_vel.png" border="0" align="center" width="250"></img></td>
</tr>
'''.format(_name, _name, _name, _name,)
        content += '\n</table>\n'
    #write_md_page(md_filename, 'Odometry Examples', content)
    write_html_include(html_filename, content)

def gen_summary_plots(examples):
    for _section, _section_examples in examples:
        for filename, _type , _name in _section_examples:
            _ds = odm_ds.DataSet(filename, _type)
            odm_ds.plot2d(_ds, filename=plot_dir+_name+'_truth_2d.png')
            odm_ds.plot_encoders(_ds, filename=plot_dir+_name+'_truth_wheels.png')
            odm_ds.plot_truth_vel(_ds, filename=plot_dir+_name+'_truth_vel.png')
        
def make_summary(examples, make_plots):
    gen_summary_mardown(examples)
    if make_plots: gen_summary_plots(examples)



#
# Regression1
#
    
def make_regression_dif_drive(examples, make_plots=False):
    #md_filename = md_dir + 'regression1.md'
    html_filename = md_dir + '_includes/regression1.html'
    content = ''
    for _section, _section_examples in examples:
        content += '''
## {}

<table style="border-collapse: collapse; border-spacing: 20px; text-align: center; border: 0px solid black;">
<tr><th>Name</th><th>Radius</th><th>Separation</th><th>Residuals</th></tr>\n'''.format(_section)
        for filename, _type , _name in _section_examples:
            _ds = odm_ds.DataSet(filename, _type)
            reg = odm_ft.Regression(_ds)
            reg.fit_wheel_radius()
            reg.fit_wheel_sep()
            if make_plots:
                reg.plot_residuals(info=_name, filename=plot_dir+_name+'_reg_dd1_res.png')
                reg.plot_wheel_radius(filename=plot_dir+_name+'_reg_dd1_wr.png')
                reg.plot_wheel_sep(filename=plot_dir+_name+'_reg_dd1_ws.png')
            comment = '{}'.format('N/A')
            content += '''<tr>
  <td style="text-align: left; padding: 15px;" valign="top" width="50">
	{} {}
  </td>
  <td markdown="1" valign="top"><img src="../plots/{}_reg_dd1_wr.png" border="0" align="center" width="250"/>\n  <br>\n\n{}\n\n  </td>
  <td markdown="1" valign="top"><img src="../plots/{}_reg_dd1_ws.png" border="0" align="center" width="250"/>\n  <br>\n\n{}\n\n  </td>
  <td valign="top"><img src="../plots/{}_reg_dd1_res.png" border="0" align="center" width="250"/></td>
</tr>
'''.format(_name, comment, _name, reg.report_fit_wheel_radius(), _name, reg.report_fit_wheel_sep(), _name)
        content += '\n</table>\n'
    #write_md_page(md_filename, 'Regression1', content)
    write_html_include(html_filename, content)
     
if __name__ == '__main__':
    #make_summary(all_examples, make_plots=True)
    make_regression_dif_drive(all_examples, make_plots=False)
