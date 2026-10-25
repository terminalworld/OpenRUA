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
  const inspection = `Check status: openrua session --name ${name} status`;
  const finished = () => write(`Session ${name} ended; your files and chat history are saved. Use openrua --resume to find it.`);
  if (ended) { finished(); return true; }
  let state;
  try { ({state} = await client.snapshot()); }
  catch (error) {
    write(`Session ${name}: we could not check its status. ${error.message}`);
    write(inspection);
    return false;
  }
  if (state.closed) { finished(); return true; }
  const answer = (await ask('Keep this session running?\n'
    + '  Enter / no  stop the agent and free up memory; keep files and chat history\n'
    + '  yes         keep this session running in the background\n> ')).trim().toLowerCase();
  if (answer === 'cancel') return false;
  if (answer === '' || answer === 'no') {
    write('Stopping the agent and releasing resources…');
    try { await client.end(); }
    catch (error) {
      write(`Session ${name}: we could not confirm that it stopped. ${error.message}`);
      write(inspection);
      return false;
    }
    finished();
  } else if (answer === 'yes') {
    write(`Left this terminal. Come back: openrua --resume · end it: openrua session --name ${name} end`);
  } else return false;
  return true;
}
