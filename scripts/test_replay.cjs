/* Browser acceptance + optional offline video capture. Run against an export. */
const fs=require('fs'),path=require('path'),assert=require('assert'),{pathToFileURL}=require('url');
const {chromium}=require('playwright');
const args=process.argv.slice(2),folder=path.resolve(args[0]||'artifacts/replay_v1');
const record=args.includes('--video');
const {snapshot,sampleStep}=require('../visualization/replay/player.js');
const payload=JSON.parse(fs.readFileSync(path.join(folder,'replay_data.json'),'utf8'));
const dbl=payload.scenes.find(s=>s.name==='double_baffle_detour');
assert(snapshot(dbl.arms.APPROACH,110).ended);
assert(!snapshot(dbl.arms.P1,110).ended);
assert.deepStrictEqual(snapshot(dbl.arms.APPROACH,110).raw,dbl.arms.APPROACH.samples.at(-1));
assert.strictEqual(sampleStep([0,.05,.1],0,1),.05);
assert.strictEqual(sampleStep([0,.05,.1],.05,-1),0);
assert.strictEqual(sampleStep([0,.05,.1],.025,1),.05);
(async()=>{
  const browser=await chromium.launch({channel:'msedge',headless:true});
  try{
    const page=await browser.newPage({viewport:{width:1440,height:1200},deviceScaleFactor:1});
    const errors=[];page.on('pageerror',e=>errors.push(e.message));
    await page.goto(pathToFileURL(path.join(folder,'index.html')).href);
    assert.strictEqual(await page.locator('#scene option').count(),5);
    await page.evaluate(()=>window.replay.setTime(85));
    await page.screenshot({path:path.join(folder,'double_baffle_85s.png'),fullPage:true});
    await page.evaluate(()=>window.replay.setTime(110));
    const s=await page.evaluate(()=>window.replay.snapshot());
    assert(s.arms.APPROACH.ended&&!s.arms.P1.ended);
    await page.screenshot({path:path.join(folder,'double_baffle_110s.png'),fullPage:true});
    await page.evaluate(()=>window.replay.setTime(0));
    await page.locator('#step').click();assert(Math.abs((await page.evaluate(()=>window.replay.snapshot().time))-.05)<1e-9);
    await page.locator('#back').click();assert.strictEqual(await page.evaluate(()=>window.replay.snapshot().time),0);
    await page.locator('#time').evaluate(e=>{e.value=80;e.dispatchEvent(new Event('input',{bubbles:true}));});
    assert.strictEqual(await page.evaluate(()=>window.replay.snapshot().time),80);
    await page.locator('#restart').click();await page.locator('#play').click();
    assert((await page.evaluate(()=>window.replay.snapshot().time))<2);
    await page.locator('#speed').selectOption('8');await page.locator('#play').click();
    await page.waitForTimeout(200);await page.locator('#play').click();
    assert((await page.evaluate(()=>window.replay.snapshot().time))>0);
    const paused=await page.evaluate(()=>window.replay.snapshot().time);
    await page.waitForTimeout(100);assert.strictEqual(await page.evaluate(()=>window.replay.snapshot().time),paused);
    for(const scene of payload.scenes){
      await page.locator('#scene').selectOption(scene.name);
      await page.evaluate(t=>window.replay.setTime(t),scene.end_time);
      const view=await page.evaluate(()=>window.replay.snapshot());
      assert(view.arms.P1.ended&&view.arms.APPROACH.ended);
      if(scene.name.includes('edge_')||scene.name.includes('zigzag'))assert(view.arms.P1.collisionDetected&&view.arms.APPROACH.collisionDetected);
    }
    await page.evaluate(()=>{window.replay.select('double_baffle_detour');window.replay.setTime(71);});
    const phase=await page.evaluate(()=>window.replay.snapshot().arms.APPROACH.phase);
    assert.strictEqual(phase,'TERMINAL_APPROACH');
    await page.setViewportSize({width:600,height:1600});
    await page.screenshot({path:path.join(folder,'narrow_layout.png'),fullPage:true});
    assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
    await page.setViewportSize({width:1440,height:1200});
    let video=null;
    if(record){
      const download=page.waitForEvent('download',{timeout:90000});
      const result=await page.evaluate(()=>window.replay.recordVideo());
      const item=await download;await item.saveAs(path.join(folder,item.suggestedFilename()));video={...result,file:item.suggestedFilename()};
      // Decode exported media; recording success alone is not playback evidence.
      const check=await browser.newPage();await check.goto(pathToFileURL(path.join(folder,item.suggestedFilename())).href);
      await check.waitForSelector('video');
      await check.locator('video').evaluate(async v=>{if(v.readyState<2)await new Promise((res,rej)=>{v.onloadeddata=res;v.onerror=rej;});});
      await check.locator('video').evaluate(v=>new Promise(resolve=>{v.onseeked=resolve;v.currentTime=25;}));
      await check.screenshot({path:path.join(folder,'video_decoded_25s.png')});
      video.decoded=await check.locator('video').evaluate(v=>({width:v.videoWidth,height:v.videoHeight,ready:v.readyState,seek_time:v.currentTime}));
      assert(video.decoded.width>0&&video.decoded.height>0);await check.close();
    }
    assert.deepStrictEqual(errors,[]);
    fs.writeFileSync(path.join(folder,'browser_acceptance.json'),JSON.stringify({passed:true,
      checks:['five_scenes','common_clock','ended_arm_frozen','step_back_forward','play_pause_speed','scrub_and_restart','collision_freeze',
              'logged_approach_phase','no_horizontal_overflow','no_browser_errors'],video},null,2));
    console.log(JSON.stringify({passed:true,video}));
  }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
