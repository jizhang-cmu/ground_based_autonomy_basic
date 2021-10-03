#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import ast
import operator as op
import rospy
import traceback
from sensor_msgs.msg import Joy


class RestrictedEvaluator(object):
    def __init__(self):
        self.operators = {
            ast.Add: op.add,
            ast.Sub: op.sub,
            ast.Mult: op.mul,
            ast.Div: op.truediv,
            ast.BitXor: op.xor,
            ast.USub: op.neg,
        }
        self.functions = {
            'abs': lambda x: abs(x),
            'max': lambda *x: max(*x),
            'min': lambda *x: min(*x),
        }

    def _reval_impl(self, node, variables):
        if isinstance(node, ast.Num):
            return node.n
        elif isinstance(node, ast.BinOp):
            op = self.operators[type(node.op)]
            return op(self._reval_impl(node.left, variables),
                      self._reval_impl(node.right, variables))
        elif isinstance(node, ast.UnaryOp):
            op = self.operators[type(node.op)]
            return op(self._reval_impl(node.operand, variables))
        elif isinstance(node, ast.Call) and node.func.id in self.functions:
            func = self.functions[node.func.id]
            args = [self._reval_impl(n, variables) for n in node.args]
            return func(*args)
        elif isinstance(node, ast.Name) and node.id in variables:
            return variables[node.id]
        elif isinstance(node, ast.Subscript) and node.value.id in variables:
            var = variables[node.value.id]
            idx = node.slice.value.n
            try:
                return var[idx]
            except IndexError:
                raise IndexError("Variable '%s' out of range: %d >= %d" % (node.value.id, idx, len(var)))
        else:
            raise TypeError("Unsupported operation: %s" % node)

    def reval(self, expr, variables):
        expr = str(expr)
        if len(expr) > 1000:
            raise ValueError("The length of an expression must not be more than 1000 characters")
        try:
            return self._reval_impl(ast.parse(expr, mode='eval').body, variables)
        except Exception as e:
            rospy.logerr(traceback.format_exc())
            raise e


class JoyRemap(object):
    def __init__(self):
        self.evaluator = RestrictedEvaluator()
        self.mappings = self.load_mappings("~mappings")
        self.warn_remap("joy_out")
        self.pub = rospy.Publisher(
            "joy_out", Joy, queue_size=1)
        self.warn_remap("joy_in")
        self.sub = rospy.Subscriber(
            "joy_in", Joy, self.callback,
            queue_size=rospy.get_param("~queue_size", None))

    def load_mappings(self, ns):
        btn_remap = rospy.get_param(ns + "/buttons", [])
        axes_remap = rospy.get_param(ns + "/axes", [])
        rospy.loginfo("loaded remapping: %d buttons, %d axes" % (len(btn_remap), len(axes_remap)))
        return {"buttons": btn_remap, "axes": axes_remap}

    def warn_remap(self, name):
        if name == rospy.remap_name(name):
            rospy.logwarn("topic '%s' is not remapped" % name)

    def callback(self, in_msg):
        out_msg = Joy(header=in_msg.header)
        map_axes = self.mappings["axes"]
        map_btns = self.mappings["buttons"]
        out_msg.axes = [0.0] * len(map_axes)
        out_msg.buttons = [0] * len(map_btns)
        in_dic = {"axes": in_msg.axes, "buttons": in_msg.buttons}
        for i, exp in enumerate(map_axes):
            try:
                out_msg.axes[i] = self.evaluator.reval(exp, in_dic)
            except NameError as e:
                rospy.logerr("You are using vars other than 'buttons' or 'axes': %s" % e)
            except UnboundLocalError as e:
                rospy.logerr("Wrong form: %s" % e)
            except Exception as e:
                raise e

        for i, exp in enumerate(map_btns):
            try:
                if self.evaluator.reval(exp, in_dic) > 0:
                    out_msg.buttons[i] = 1
            except NameError as e:
                rospy.logerr("You are using vars other than 'buttons' or 'axes': %s" % e)
            except UnboundLocalError as e:
                rospy.logerr("Wrong form: %s" % e)
            except Exception as e:
                raise e

        self.pub.publish(out_msg)


if __name__ == '__main__':
    rospy.init_node("joy_remap")
    n = JoyRemap()
    rospy.spin()
