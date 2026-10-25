"""Native defaults and persisted identifiers retain the selected provider's meaning."""
import pytest

from openrua import agents
from openrua.plugins.agents.claude_conversation import ClaudeConversation
from openrua.plugins.agents.codex_conversation import CodexConversation


@pytest.mark.parametrize('name,old,native', [
    ('claude-code', 'auto', 'bypassPermissions'), ('claude-code', 'auto-review', 'auto'),
    ('claude-code', 'ask', 'default'), ('codex', 'auto', 'never'),
    ('codex', 'auto-review', 'auto_review'), ('codex', 'ask', 'untrusted'),
])
def test_restoring_older_permissions_does_not_reinterpret_a_bypass(name, old, native):
    adapter = agents.get(name)
    assert agents.saved_permissions(adapter, {'permissions': old}) == native
    assert agents.saved_permissions(adapter, {'permissions_format': 'native'}, {
        'settings': {'permissions': old}, 'setting_choices': {'permissions': [
            {'value': 'ask'}, {'value': 'auto-review'}, {'value': 'auto'}]}}) == native


def test_native_auto_is_unambiguous_in_new_facts_and_current_native_catalogs():
    adapter = agents.get('claude-code')
    assert agents.saved_permissions(adapter, {'permissions': 'auto', 'permissions_format': 'native'}) == 'auto'
    assert agents.saved_permissions(adapter, {'permissions': 'auto'}, {
        'settings': {'permissions': 'auto'}, 'setting_choices': {'permissions': adapter.conversation(
            'box', 'model', '').protocol.PERMISSIONS}}) == 'auto'


def test_provider_defaults_are_native_review_not_full_access():
    claude = agents.get('claude-code').conversation('box', 'model', '')
    assert claude.argv[claude.argv.index('--permission-mode') + 1] == 'auto'
    codex = agents.get('codex').conversation('box', 'model', '')
    assert codex.protocol.permissions == 'auto_review'
    assert codex.protocol._thread_params()['sandbox'] == 'workspace-write'
    assert codex.protocol._approval_params() == {'approvalPolicy': 'on-request', 'approvalsReviewer': 'auto_review'}
    assert 'sandbox_mode="danger-full-access"' not in codex.argv
    assert '--dangerously-bypass-approvals-and-sandbox' not in agents.get('codex').interactive_argv('box', 'model', '')
    assert 'plan' not in agents.get('codex').permission_modes


@pytest.mark.parametrize('name', ['kimi', 'zcode'])
def test_previews_do_not_claim_switchable_native_permission_or_model_controls(name):
    adapter = agents.get(name)
    assert adapter.permission_modes == () and adapter.default_permissions is None
    assert agents.saved_permissions(adapter, {'permissions': 'auto'}) is None
    assert not adapter.model_selection


def test_claude_catalog_is_read_from_initialization_and_a_mismatched_mode_fails():
    protocol = ClaudeConversation('native')
    protocol.model = 'selected'
    request = protocol.begin().outbound[0]
    response = {'type': 'control_response', 'response': {'request_id': request['request_id'],
        'subtype': 'success', 'response': {'current_permission_mode': 'auto',
            'models': [{'value': 'native-dynamic-model', 'displayName': 'Native label',
                        'description': 'Native description', 'resolvedModel': 'native-release'}]}}}
    update = protocol.receive(response)
    catalog = next(e.data for e in update.events if e.data.get('setting') == 'model')
    assert catalog['value'] == 'selected' and catalog['choices'][0]['value'] == 'native-dynamic-model'
    assert catalog['choices'][0]['resolved_model'] == 'native-release'
    protocol = ClaudeConversation('native')
    protocol.begin()
    response['response']['response']['current_permission_mode'] = 'bypassPermissions'
    with pytest.raises(ValueError, match='differs from requested'):
        protocol.receive(response)
    assert not protocol.can_submit


def test_codex_metadata_query_does_not_create_a_thread_or_submit_a_turn():
    connection = agents.get('codex').model_catalog('box', 'model', '')
    protocol = connection.protocol
    initialize = protocol.begin().outbound[0]
    update = protocol.receive({'id': initialize['id'], 'result': {}})
    assert [frame['method'] for frame in update.outbound] == ['initialized', 'model/list']
    query = update.outbound[-1]
    update = protocol.receive({'id': query['id'], 'result': {'data': [
        {'model': 'native-model', 'displayName': 'Native model', 'defaultReasoningEffort': 'medium',
         'supportedReasoningEfforts': [{'reasoningEffort': 'medium'}]}]}})
    assert not update.outbound and not protocol.can_submit
    assert update.events[0].data['choices'][0]['efforts'] == [
        {'value': 'medium', 'label': 'medium', 'description': ''}]
