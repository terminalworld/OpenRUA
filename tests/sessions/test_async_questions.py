"""Selectable native async questions use the existing durable message queue."""

import asyncio
import json

import pytest

from openrua.agents.conversation import Conversation
from openrua.plugins.agents.codex_conversation import CodexConversation
from openrua.sessions.core import Session, initial_state
from openrua.sessions.execution import Execution
from openrua.sessions.store import SQLiteStore
from tests.sessions.test_execution import until


class NativeTransport:
    def __init__(self):
        self.frames = asyncio.Queue()
        self.writes = []
        self.fail_steer = False

    async def read(self):
        return await self.frames.get()

    async def write(self, frame):
        self.writes.append(frame)
        method = frame.get('method')
        if method == 'turn/steer' and self.fail_steer:
            raise OSError('connection lost after writing the answer')
        result = {'initialize': {}, 'thread/start': {'thread': {'id': 'thread-1'}},
                  'turn/start': {'turn': {'id': 'turn-1'}}}.get(method)
        if result is not None:
            await self.frames.put({'id': frame['id'], 'result': result})

    async def close(self):
        pass


async def asking(tmp_path):
    store = SQLiteStore(tmp_path / 'session.sqlite', initial_state())
    session = Session(store)
    transport = NativeTransport()

    async def start_transport():
        return transport

    owner = Execution(session, Conversation(['fake'], CodexConversation('model', permissions='never')),
                      start_transport, tmp_path / 'owner.lock')
    await owner.start()
    await until(lambda: store.snapshot()['state']['connected'])
    first = await owner.command('enqueue', client_id='terminal', request_id='one', text='Inspect the table')
    await until(lambda: store.snapshot()['state']['messages'][0]['status'] == 'running')
    await transport.frames.put({'method': 'item/completed', 'params': {
        'threadId': 'thread-1', 'turnId': 'turn-1', 'item': {
            'type': 'agentMessage', 'id': 'call_question', 'text': 'Which target?',
            'delivery': 'async', 'phase': 'final_answer', 'memoryCitation': None,
            'questions': [{'title': 'Which target?', 'options': ['Cup', 'Bowl']},
                          {'title': 'Where should it go?', 'options': []}]}}})
    await until(lambda: bool(store.snapshot()['state']['requests']))
    request = next(iter(store.snapshot()['state']['requests'].values()))
    answers = {q['id']: [value] for q, value in zip(request['questions'], ['Cup', 'On the tray'])}
    return store, session, transport, owner, first, request, answers


def test_async_answer_reaches_native_steer_once_and_is_readable_in_history(tmp_path):
    async def run():
        store, _, native, owner, first, request, answers = await asking(tmp_path)
        try:
            assert not store.snapshot()['state']['paused']
            before = store.snapshot()
            with pytest.raises(ValueError, match='every question'):
                await owner.command('respond', request_id=request['request_id'], answers={})
            assert store.snapshot() == before
            accepted = await owner.command('respond', request_id=request['request_id'], answers=answers)
            state = store.snapshot()['state']
            assert state['active'] == first['id'] and not state['paused'] and not state['requests']
            assert state['messages'][1]['display_text'] == 'Which target?\nAnswer: Cup\n\nWhere should it go?\nAnswer: On the tray'
            steers = [frame for frame in native.writes if frame.get('method') == 'turn/steer']
            assert len(steers) == 1
            assert steers[0]['params']['expectedTurnId'] == 'turn-1'
            text = steers[0]['params']['input'][0]['text']
            assert json.loads(text.splitlines()[1]) == [
                {'questionItemId': q['id'], 'question': q['text'], 'answer': answers[q['id']][0]}
                for q in request['questions']]
            assert not any(frame.get('id') == 'call_question' for frame in native.writes)
            with pytest.raises(ValueError, match='no longer waiting'):
                await owner.command('respond', request_id=request['request_id'], answers=answers)
            assert len([frame for frame in native.writes if frame.get('method') == 'turn/steer']) == 1
            await native.frames.put({'id': steers[0]['id'], 'result': {'turnId': 'turn-1'}})
            await until(lambda: store.snapshot()['state']['messages'][1]['status'] == 'steered')
            events = store.events()
            acceptance = next(event for event in events if event['kind'] == 'message_accepted'
                              and event['data']['message']['id'] == accepted['id'])
            assert acceptance['data']['message']['display_text'] == state['messages'][1]['display_text']
            assert any(event['kind'] == 'input_reply_queued' and event['data']['message_id'] == accepted['id']
                       for event in events)
        finally:
            await owner.close()
            store.close()
    asyncio.run(run())


def test_answer_racing_a_finished_turn_is_sent_once_as_the_next_turn(tmp_path):
    async def run():
        store, _, native, owner, _, request, answers = await asking(tmp_path)
        try:
            accepted = await owner.command('respond', request_id=request['request_id'], answers=answers)
            steer = next(frame for frame in native.writes if frame.get('method') == 'turn/steer')
            await native.frames.put({'method': 'turn/completed', 'params': {
                'threadId': 'thread-1', 'turn': {'id': 'turn-1', 'status': 'completed'}}})
            await native.frames.put({'id': steer['id'], 'error': {'message': 'no active turn to steer'}})
            await until(lambda: len([f for f in native.writes if f.get('method') == 'turn/start']) == 2)
            starts = [f for f in native.writes if f.get('method') == 'turn/start']
            assert starts[1]['params']['input'] == steer['params']['input']
            assert store.snapshot()['state']['active'] == accepted['id']
            assert len(store.snapshot()['state']['messages']) == 2
        finally:
            await owner.close()
            store.close()
    asyncio.run(run())


def test_uncertain_async_answer_is_retained_without_replay(tmp_path):
    async def run():
        store, _, native, owner, _, request, answers = await asking(tmp_path)
        try:
            native.fail_steer = True
            await owner.command('respond', request_id=request['request_id'], answers=answers)
            state = store.snapshot()['state']
            assert not state['connected'] and state['paused'] and not state['requests']
            assert state['messages'][1]['status'] == 'unknown'
            with pytest.raises(ValueError, match='not connected'):
                await owner.command('respond', request_id=request['request_id'], answers=answers)
            await owner.wake()
            assert len([f for f in native.writes if f.get('method') == 'turn/steer']) == 1
        finally:
            await owner.close()
            store.close()
    asyncio.run(run())


def test_editing_an_answer_queued_during_interrupt_uses_visible_text(tmp_path):
    async def run():
        store, session, native, owner, first, request, answers = await asking(tmp_path)
        try:
            # An interruption keeps the answer queued until the current turn ends.
            session.interrupt(first['id'])
            accepted = await owner.command('respond', request_id=request['request_id'], answers=answers)
            queued = store.snapshot()['state']['messages'][1]
            assert queued['status'] == 'queued' and queued['display_text']
            session.edit(accepted['id'], 'Put the bowl on the tray instead', queued['revision'])
            edited = store.snapshot()['state']['messages'][1]
            assert 'display_text' not in edited and edited['text'] == 'Put the bowl on the tray instead'
            await native.frames.put({'method': 'turn/completed', 'params': {
                'threadId': 'thread-1', 'turn': {'id': 'turn-1', 'status': 'completed'}}})
            await until(lambda: store.snapshot()['state']['messages'][0]['status'] == 'completed')
            with pytest.raises(ValueError, match='no longer waiting'):
                await owner.command('respond', request_id=request['request_id'], answers=answers)
        finally:
            await owner.close()
            store.close()
    asyncio.run(run())


@pytest.mark.parametrize('status', ['completed', 'interrupted'])
def test_native_turn_completion_or_cancellation_expires_unanswered_questions(tmp_path, status):
    async def run():
        store, _, native, owner, _, request, answers = await asking(tmp_path)
        try:
            await native.frames.put({'method': 'turn/completed', 'params': {
                'threadId': 'thread-1', 'turn': {'id': 'turn-1', 'status': status}}})
            await until(lambda: not store.snapshot()['state']['requests'])
            with pytest.raises(ValueError, match='no longer waiting'):
                await owner.command('respond', request_id=request['request_id'], answers=answers)
            assert len(store.snapshot()['state']['messages']) == 1
            assert not any(frame.get('method') == 'turn/steer' for frame in native.writes)
        finally:
            await owner.close()
            store.close()
    asyncio.run(run())
