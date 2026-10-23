import {createInterface} from 'node:readline/promises';

async function askChoice(prompt) {
  const input = createInterface({input: process.stdin, output: process.stdout});
  input.on('SIGINT', () => input.close());
  try { return await input.question(prompt); }
  catch { return ''; }
  finally { input.close(); }
}

// Only an acknowledged end or a current snapshot can establish session status.
// Leaving the client never starts, resumes or stops resources on its own.
export async function leave(name, client, {ended = false, ask = askChoice, write = console.log} = {}) {
  const inspection = `Inspect: openrua session --name ${name} status`;
  const finished = () => write(`Session ${name} ended; its files are kept (openrua --resume shows it).`);
  if (ended) { finished(); return true; }
  let state;
  try { ({state} = await client.snapshot()); }
  catch (error) {
    write(`Session ${name}: status unconfirmed. Resources may still be running. ${error.message}`);
    write(inspection);
    return false;
  }
  if (state.closed) { finished(); return true; }
  write(`Session ${name} is open. Leaving this terminal keeps its resources in the background.`);
  const answer = await ask('  Enter  leave it in the background\n'
    + '  e      end the session and stop owned resources; keep files\n> ');
  if (answer.trim().toLowerCase() === 'e') {
    write('Ending session… stopping the agent and owned resources.');
    try { await client.end(); }
    catch (error) {
      write(`Session ${name}: shutdown unconfirmed. ${error.message}`);
      write(inspection);
      return false;
    }
    finished();
  } else {
    write(`Left this terminal. Come back: openrua --resume · end it: openrua session --name ${name} end`);
  }
  return true;
}
