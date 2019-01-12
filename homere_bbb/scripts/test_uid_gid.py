#!/usr/bin/env python
import os


if __name__ == '__main__':
    print('uid {}'.format(os.getuid()))
    print('gid {}'.format(os.getgid()))
    print('groups {}'.format(os.getgroups()))
