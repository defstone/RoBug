# Copyright (c) 2026 RobotsForAll
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

class rbcom:

    def __init__(self, MsgQueue, RplyQueue):
        self.MsgQueue = MsgQueue
        self.RplyQueue = RplyQueue
        self.strMsg = ''

    def check_inbox(self):
        strMsg = '_NOP_'
        if len(self.MsgQueue) > 0:
            strMsg = self.MsgQueue.pop()
            print('motion server received: ', strMsg)
        return strMsg

    def command_complete(self):
        self.RplyQueue.append('DONE')

    def command_unknown(self, strCmd):
        print('unknown command: ', strCmd)
        self.RplyQueue.append('UNKONWN_CMD')

    def subcommand_unknown(self):
        self.RplyQueue.append('UNKONWN_SUBCMD')

    def get_command(self):

        strMsg = self.check_inbox()

        # transition stop pose (neutral stance) -> start pose
        if   strMsg == 'START_POSE_FWD':
            strCmd = '_cmd_START_STEP_'
            strSubCmd = '_cmd_FWD_'

        elif strMsg == 'START_POSE_BWD':
            strCmd = '_cmd_START_STEP_'
            strSubCmd = '_cmd_BWD_'

        # transition start pose -> move
        elif strMsg == 'RESUME_FWD':
            strCmd = '_cmd_RESUME_'
            strSubCmd = '_cmd_FWD_'

        elif strMsg == 'RESUME_BWD':
            strCmd = '_cmd_RESUME_'
            strSubCmd = '_cmd_BWD_'

        # transition moving -> stop pose (neutral stance)
        elif strMsg == 'STOP_POSE_FWD':
            strCmd = '_cmd_STOP_STEP_'
            strSubCmd = '_cmd_FWD_'

        elif strMsg == 'STOP_POSE_BWD':
            strCmd = '_cmd_STOP_STEP_'
            strSubCmd = '_cmd_BWD_'

        # scripted actions
        elif strMsg == 'TURN_LFT':
            strCmd = '_cmd_TURN_LFT_'
            strSubCmd = '_cmd_NA_'
            
        elif strMsg == 'TURN_RGT':
            strCmd = '_cmd_TURN_RGT_'
            strSubCmd = '_cmd_NA_'            
            
        elif strMsg == 'SIT_DOWN':
            strCmd = '_cmd_SIT_DOWN_'
            strSubCmd = '_cmd_NA_'

        elif strMsg == 'STAND_UP':
            strCmd = '_cmd_STAND_UP_'
            strSubCmd = '_cmd_NA_'

        # special case handling
        elif strMsg == '_NOP_':
            strCmd = '_cmd_NOP_'
            strSubCmd = '_cmd_NOP_'

        elif strMsg == 'EXIT':
            strCmd = '_cmd_EXIT_'
            strSubCmd = '_cmd_NOP_'

        else:
            strCmd = '_cmd_UNKNOWN_'
            strSubCmd = '_cmd_UNKNOWN_'
        if strCmd != '_cmd_NOP_':
            print(f'translation: {strCmd}, {strSubCmd}')
        return strCmd, strSubCmd
