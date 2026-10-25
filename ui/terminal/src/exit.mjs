import {createInterface} from 'node:readline/promises';

async function askChoice(prompt) {
  const input = createInterface({input: process.stdin, output: process.stdout});
  input.on('SIGINT', () => input.close());
  try { return await input.question(prompt); }
  catch { return 'cancel'; }
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
  const answer = (await ask('Keep execution resources running?\n'
    + '  Enter / no  stop owned resources; keep files and history\n'
    + '  yes         keep execution running in the background\n> ')).trim().toLowerCase();
  if (answer === 'cancel') return false;
  if (answer === '' || answer === 'no') {
    write('Ending session… stopping the agent and owned resources.');
    try { await client.end(); }
    catch (error) {
      write(`Session ${name}: shutdown unconfirmed. ${error.message}`);
      write(inspection);
      return false;
    }
    finished();
  } else if (answer === 'yes') {
    write(`Left this terminal. Come back: openrua --resume · end it: openrua session --name ${name} end`);
  } else return false;
  return true;
}
