"""Drive native metadata streams without submitting a turn or retaining a child."""
import sys

import pytest

from openrua.agents.conversation import Conversation
from openrua.cli.model_catalog import read
from openrua.errors import UnavailableError
from openrua.plugins.agents.claude_conversation import ClaudeConversation


def test_native_metadata_only_returns_choices_without_submitting_a_prompt():
    program = '''
import json,sys
request=json.loads(sys.stdin.readline())
assert request['request']['subtype']=='initialize'
print(json.dumps({'type':'control_response','response':{'request_id':request['request_id'],
 'subtype':'success','response':{'models':[{'value':'native-model','displayName':'Native model'}]}}}),flush=True)
assert not sys.stdin.read()
'''
    rows = read(Conversation([sys.executable, '-u', '-c', program], ClaudeConversation('native')))
    assert rows[0]['value'] == 'native-model'


def test_native_early_exit_and_timeout_are_visible():
    with pytest.raises(UnavailableError, match='ended before'):
        read(Conversation([sys.executable, '-c', 'pass'], ClaudeConversation('native')))
    with pytest.raises(UnavailableError, match='timed out'):
        read(Conversation([sys.executable, '-c', 'import time; time.sleep(60)'],
                          ClaudeConversation('native')), timeout=.05)
