// Builds report/Report.html (self-contained math) and report/Report.pdf from report_template.html.
// Needs: npm install katex@0.16.11 playwright   (run from any folder that has them in node_modules)
// Usage:  node report/src/build_report.js
const fs = require("fs");
const path = require("path");
const katex = require("katex");
const { chromium } = require("playwright");

const here = __dirname;
const outDir = path.resolve(here, "..");
const katexDir = path.dirname(require.resolve("katex/dist/katex.min.css"));

const decode = (s) => s.replace(/&lt;/g, "<").replace(/&gt;/g, ">").replace(/&amp;/g, "&");
const render = (tex, displayMode) =>
  katex.renderToString(decode(tex), { displayMode, throwOnError: true, strict: false,
    macros: { "\\dddot": "\\overset{\\scriptscriptstyle\\bullet\\bullet\\bullet}{#1}" } });

// KaTeX CSS with its woff2 fonts embedded, so the HTML works on its own.
const css = fs.readFileSync(path.join(katexDir, "katex.min.css"), "utf8")
  .replace(/src:url\(fonts\/([^)]+?\.woff2)\) format\("woff2"\),[^;}]*/g, (_, f) =>
    `src:url(data:font/woff2;base64,${fs.readFileSync(path.join(katexDir, "fonts", f)).toString("base64")}) format("woff2")`);

let html = fs.readFileSync(path.join(here, "report_template.html"), "utf8")
  .replace("/*KATEX_CSS*/", css)
  .replace(/\$\$([\s\S]+?)\$\$/g, (_, tex) => render(tex, true))
  .replace(/\\\(([\s\S]+?)\\\)/g, (_, tex) => render(tex, false));

const htmlPath = path.join(outDir, "Report.html");
fs.writeFileSync(htmlPath, html);

(async () => {
  const browser = await chromium.launch();
  const page = await browser.newPage();
  await page.goto("file://" + htmlPath, { waitUntil: "networkidle" });
  await page.pdf({
    path: path.join(outDir, "Report.pdf"), format: "A4", printBackground: true, preferCSSPageSize: true,
    displayHeaderFooter: true, headerTemplate: "<span></span>",
    footerTemplate: '<div style="width:100%;font:8px Inter,sans-serif;color:#888;text-align:center;">' +
                    'AR523 Assignment 2 · page <span class="pageNumber"></span> of <span class="totalPages"></span></div>',
  });
  await browser.close();
  console.log("wrote", htmlPath, "and Report.pdf");
})();
